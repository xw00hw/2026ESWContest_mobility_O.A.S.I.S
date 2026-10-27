#!/bin/bash
# Final compute pipeline (v8 = OASIS_FALLBACK v2 actuator + BeamNG.drive Collector inputs;
# OASIS_VERSION=v7 OASIS_OVERLAYS=overlays_v7.txt reproduces v7).
# Idempotent: every stage is skipped when its output exists, so re-running after an interruption continues where it
# stopped. Logs: models/<version>/<stage>.log.   bash sim/v3/final/run_final.sh [stage]
# stages: predictor audit rule train select eval export tests ablation body trace
set -e
source "$(dirname "$0")/env.sh"                 # OASIS_ROOT, OASIS_ENV, OASIS_CALIB, OASIS_L1_SEQ, OASIS_DR_OVERLAY
ROOT=$OASIS_ROOT; W=$(command -v cygpath >/dev/null && cygpath -m "$ROOT" || echo "$ROOT")
V=${OASIS_VERSION:-v8}; M="$ROOT/models/$V"; MW="$W/models/$V"; mkdir -p "$M"
TMP=${TMPDIR:-/tmp}; [ -d "$TMP" ] || TMP=$HOME   # cwd for python -P (outside the data folders)
only=${1:-all}; run() { [ "$only" = all ] || [ "$only" = "$1" ]; }
say() { echo "[$(date +%H:%M:%S)] $*"; }
py() { (cd "$TMP" && python -P "$@"); }
json() { python -P -c "import json, sys; d = json.load(open(sys.argv[1])); print(eval(sys.argv[2]))" "$1" "$2"; }

if run predictor && [ ! -f "$M/predictor.npz" ]; then say predictor
  py "$W/sim/v3/final/train_predictor.py" --out "$MW" --n 1200 --nval 240 --epochs 40 --workers 12 > "$M/train_predictor.out" 2>&1; fi
if run audit && [ ! -f "$M/audit.json" ]; then say audit
  py "$W/sim/v3/audit_v6.py" --out "$MW/audit.json" --rule "$MW/rule.json" > "$M/audit.log" 2>&1 || true; tail -n 1 "$M/audit.log"; fi
if run rule && [ ! -f "$M/rule.json" ]; then say rule grid
  py "$W/sim/v3/tune_rule.py" --grid --n 48 --out "$MW/rule.json" > "$M/tune_rule.log" 2>&1; fi
GAIN=$(json "$MW/rule.json" "d['best']['gain']"); LEAD=$(json "$MW/rule.json" "d['best']['lead_idx']")
# SAC seeds and trainer: OASIS_TRAIN_PY = an interpreter whose torch has CUDA (sac_asym.py --device auto uses it;
# this PC: .venv-gpu, 4 seeds x 2 threads ~19 ms/step each); default the CPU python, 2 seeds x 5 threads (~16 ms/step)
SEEDS=${OASIS_SEEDS:-"0 1"}; TPY=${OASIS_TRAIN_PY:-python}; TH=${OASIS_TRAIN_THREADS:-5}; NS=$(echo $SEEDS | wc -w)
if run train; then
  cd "$ROOT/sim/rl"
  for s in $SEEDS; do
    if ! grep -q "^done ${V}_s$s" "runs/${V}_s$s.out" 2>/dev/null; then say "train ${V}_s$s (prior gain $GAIN lead $LEAD)"
      mkdir -p runs
      # v8: start from the tuned rule and adjust -- critic-only for the first 20k steps, small exploration, correction
      # <= 0.25 and pulled toward 0 -- on the evaluation score as the reward (reward_score.yaml). v7: --res_scale 0.5
      X="--res_scale 0.25 --actor_delay 20000 --init_log_std -1.0 --warmup_noise 0.1 --res_l2 0.02"; [ "$V" = v7 ] && X="--res_scale 0.5"
      nohup "$TPY" sac_asym.py --steps 200000 --tag "${V}_s$s" --residual $X --ff_gain "$GAIN" --ff_lead_idx "$LEAD" \
        --caps_t 2.0 --caps_s 0.5 --seed "$s" --threads "$TH" > "runs/${V}_s$s.out" 2>&1 &
    fi
  done
  wait
  for s in $SEEDS; do grep -q "^done ${V}_s$s" "runs/${V}_s$s.out" || { say "${V}_s$s did not finish"; exit 1; }; done
fi
if run select; then cd "$ROOT/sim/rl"
  # v8: the rule's 48 validation episodes (levels 1 / 2) with the prior rule scored alongside; v7: 8 episodes at level 2
  SEL="--n 48 --levels 1 2 --prior $GAIN $LEAD --workers $((12 / NS))"; [ "$V" = v7 ] && SEL="--n 8 --level 2.0"
  for s in $SEEDS; do [ -f "runs/${V}_s$s/actor_best.pt" ] || { say "select ${V}_s$s"; python select_ckpt.py "${V}_s$s" --criterion score --last 6 $SEL > "runs/${V}_s${s}_select.out" 2>&1 & }; done; wait; fi
if run eval && [ ! -f "$ROOT/sim/v3/eval_${V}_summary.json" ]; then say eval bank; cd "$ROOT/sim/rl"
  RUNS=$(for s in $SEEDS; do printf "%s_s%s " "$V" "$s"; done); [ "$V" != v7 ] && [ -f runs/v7_s1/actor_best.pt ] && RUNS="$RUNS v7_s1"
  python evaluate.py --n 24 --levels 0 2 --runs $RUNS --rule "../../models/$V/rule.json" --out "eval_bank_$V.csv" --workers 12 > "runs/eval_$V.out" 2>&1
  py "$W/sim/v3/summarize_eval.py" "$W/sim/rl/eval_bank_$V.csv" --ref rule_tuned --out "$W/sim/v3/eval_${V}_summary.json" > "$M/eval_summary.log" 2>&1; fi
if run export && [ ! -f "$ROOT/deploy/model_$V/manifest.json" ]; then say export
  # v8: the run with the best VALIDATION J (select_ckpt.json, rule's validation episodes); v7: eval-bank J of s0 / s1
  BEST=$(python -P -c "
import json, os, sys; v, root, seeds = sys.argv[1], sys.argv[2], sys.argv[3].split()
runs = [f'{v}_s{s}' for s in seeds]; sel = {r: os.path.join(root, 'sim', 'rl', 'runs', r, 'select_ckpt.json') for r in runs}
if v != 'v7' and all(os.path.exists(p) for p in sel.values()):
    J = {r: (lambda d: d['table'][d['best']]['score'])(json.load(open(p))) for r, p in sel.items()}
else:
    d = json.load(open(os.path.join(root, 'sim', 'v3', f'eval_{v}_summary.json')))['eval_bank']
    J = {r: sum(d[l]['sac_' + r]['J'] for l in d) / len(d) for r in runs}
print(max(J, key=J.get))" "$V" "$W" "$SEEDS")
  echo "$BEST" > "$M/best_run.txt"
  (cd "$ROOT/sim/rl" && python export_actor.py "runs/$BEST/actor_best.pt") > "$M/export.log" 2>&1
  (cd "$ROOT" && python -P deploy/export/export_actor.py "sim/rl/runs/$BEST/actor_best.npz" --out "deploy/model_$V" --calib "$OASIS_CALIB" --overlay "$OASIS_DR_OVERLAY") >> "$M/export.log" 2>&1
  (cd "$ROOT" && python -P deploy/export/make_calib_set.py --model "deploy/model_$V" --episodes 24) >> "$M/export.log" 2>&1; fi
BEST=$(cat "$M/best_run.txt" 2>/dev/null || echo "${V}_s1")
if run tests; then say runtime tests
  (cd "$TMP" && for t in test_runtime test_app_fb; do OASIS_TEST_MODEL_V3="$W/deploy/model_$V" python -P "$W/deploy/runtime/tests/$t.py" 2>&1 | grep -E "^(PASS|FAIL)" | sed "s/^/$t: /"; done) > "$M/runtime_tests.txt" || true; cat "$M/runtime_tests.txt"; fi
if run ablation && [ ! -f "$M/obs_ablation.json" ]; then say ablation
  py "$W/sim/v3/obs_ablation.py" --run "$BEST" --model "$W/deploy/model_$V" --n 24 --out "$MW/obs_ablation.json" > "$M/obs_ablation.out" 2>&1; fi
if run body && [ ! -f "$M/body_type_eval.json" ]; then say body types
  py "$W/sim/v3/body_type_eval.py" --runs "$BEST" --rule "$MW/rule.json" --n 24 --out "$MW/body_type_eval.csv" > "$M/body_type_eval.out" 2>&1; fi
if run trace; then
  for s in 1000 1001 1003 1004; do [ -f "$M/trace_$s.json" ] && continue; say "trace $s"
    k=$(python -P -c "print(['urban_turn', 'lane_change', 'slalom', 'l1_replay'][$s % 4])")
    py "$W/sim/v3/scenario_trace.py" --run "$BEST" --rule "$MW/rule.json" --seed "$s" --kind "$k" --window 14 --out "$MW/trace_$s.json" > "$M/trace_$s.out" 2>&1
  done; fi
say done
