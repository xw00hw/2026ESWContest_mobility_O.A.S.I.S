#!/bin/bash
# Environment of the final simulator (v8): source it from anywhere.   source sim/v3/final/env.sh
# OASIS_OVERLAYS=overlays_v7.txt selects the v7 overlay list (FW-proposal actuator) instead of overlays.txt.
# OASIS_ROOT = the RL project root (local Active_bolster, or "3_RL_Control" in the team repo).
_here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd); export OASIS_ROOT=$(cd "$_here/../../.." && pwd)
_win() { command -v cygpath >/dev/null && cygpath -m "$1" || echo "$1"; }
_sep=':'; command -v cygpath >/dev/null && _sep=';'                  # os.pathsep of the Python that reads it
export OASIS_ENV=v3
export OASIS_CALIB=${OASIS_CALIB:-$(_win "$OASIS_ROOT/sim/v3/final/calib_v7.json")}
# derived from the raw real drive (not in git): python sim/v2/make_l1_seq.py --run <refined_drive/01_refined/run2> --out models/v5/l1_seq.csv
export OASIS_L1_SEQ=${OASIS_L1_SEQ:-$(_win "$OASIS_ROOT/models/v5/l1_seq.csv")}
OASIS_DR_OVERLAY=""
while read -r p; do case "$p" in ''|\#*) continue;; esac; OASIS_DR_OVERLAY="${OASIS_DR_OVERLAY:+$OASIS_DR_OVERLAY$_sep}$(_win "$OASIS_ROOT/$p")"; done < "$_here/${OASIS_OVERLAYS:-overlays.txt}"
export OASIS_DR_OVERLAY
