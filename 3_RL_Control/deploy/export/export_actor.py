"""Turn a trained actor (sim/rl export: actor_best.npz) into Pi deployables.

    python deploy/export/export_actor.py sim/rl/runs/v2_resid_smooth/actor_best.npz --out deploy/model

writes
  actor.npz              copy of the source (numpy backend + host-side head for Hailo)
  actor_weights.h        C arrays for oasis_rt/actor_core.c  (W as [out][in], 1/std tiled over K+1)
  actor_core_fc.onnx     MLP core, normalised input [1, N_IN] -> [1, 4]          (Gemm)
  actor_core_conv.onnx   same as 1x1 convolutions, input [1, N_IN, 1, 1]          (Hailo-friendly)
  obs_layout.py          verbatim copy of the contract the actor was trained with (OD 46: sim/v2/obs_layout.py,
                         OD 57: sim/v3/obs_layout_v3.py); the runtime imports THIS file (sha checked)
  manifest.json          dims, obs contract version, sha256 of every file, ONNX-vs-numpy error

The tanh / residual prior / clip stay on the CPU in every backend, so all three backends
compute exactly the same function as sim/rl/deploy_actor.DeployedActor.act.
ONNX is written straight from the npz (no torch); opset 13, input name obs_norm, output core_out.
"""
import argparse, hashlib, json, os, shutil, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
import importlib.util
import yaml

CONTRACTS = {46: os.path.join(ROOT, 'sim', 'v2', 'obs_layout.py'), 57: os.path.join(ROOT, 'sim', 'v3', 'obs_layout_v3.py')}


def contract(od):
    if od not in CONTRACTS: raise SystemExit(f'no observation contract with OD {od}')
    spec = importlib.util.spec_from_file_location('obs_contract', CONTRACTS[od])
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return CONTRACTS[od], m


def _merge(base, over):
    """same rule as sim/bolster_env.deep_merge: an overlay replaces only the keys it names"""
    out = dict(base)
    for k, v in (over or {}).items():
        out[k] = _merge(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) and 'dist' not in v else v
    return out


def model_defaults(od, calib=None, overlay=None):
    """what the simulator assumed, for the runtime defaults (overridden by pad_calib.json). calib / overlay:
    the versioned calibration and DR overlay a pipeline model was trained with (models/<v>/)."""
    cal = json.load(open(calib or os.path.join(ROOT, 'sim', 'v2', 'calib_v2.json')))
    if od == 46:
        return dict(sim_counts_per_N=cal['G_side_counts_per_N'], channel_map={'side_R': 'A1_C1', 'side_L': 'A1_C3'})
    base = yaml.safe_load(open(os.path.join(ROOT, 'sim', 'v3', 'dr_v3.yaml')))
    v3, mode = base['v3'], base['seat_bolster'].get('actuator_mode', 'fw_event_hb')
    for ov in (overlay.split(os.pathsep) if overlay else []):
        y = yaml.safe_load(open(ov)) or {}
        v3 = _merge(v3, y.get('v3', {})); mode = (y.get('seat_bolster') or {}).get('actuator_mode', mode)
    cg = v3.get('cush_groups') or {'R': ['a0_c3'], 'L': ['a0_c1', 'a0_c2']}        # grouping the model was trained with
    cush = {'cush_R': [c.upper() for c in cg['R']], 'cush_L': [c.upper() for c in cg['L']]}
    out = dict(sim_counts_per_N=cal['G_side_counts_per_N'] / v3['zone_share_nominal'], channel_map=dict(v3['channel_map'], **cush),
               pitch_max_deg=v3['pitch_max_deg'], pitch_sign=v3['pitch_sign'], calib_zone=v3['calib_zone'])
    # the actuator the policy was trained with (the runtime checks the board's committed STM tuning against it)
    fa = v3.get('fb_a2') if mode == 'fb_a2' else None
    out['actuator'] = dict(mode=mode, firmware='OASIS_FALLBACK v2' if fa else 'OASIS_CONTROL v6 / FW proposal',
                           min_tune_rate_dps=float(min(fa['rate_dps_choices'])) if fa else None,
                           yaw_caps_deg=[float(c) for c in fa['yaw_cap_choices']] if fa else None)
    if v3.get('predictor'):                       # v6+: the a_y preview the policy was trained with (runtime default)
        out['predictor'] = dict(kind=v3['predictor']['kind'], **v3['predictor'].get('params', {}))
        if v3['predictor'].get('npz'): out['predictor']['npz_source'] = v3['predictor']['npz']   # v7: learned weights
    return out


def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()


def c_float(v):
    if not np.isfinite(v): raise ValueError(f'non-finite weight {v}')
    s = f'{float(v):.9g}'
    return (s if ('.' in s or 'e' in s) else s + '.0') + 'f'          # '8' -> '8.0f' (8f is not C)


def c_array(name, a, per_line=8):
    a = np.asarray(a, np.float32).reshape(-1)
    body = ',\n'.join('  ' + ', '.join(c_float(v) for v in a[i:i + per_line]) for i in range(0, a.size, per_line))
    return f'static const float {name}[{a.size}] __attribute__((aligned(64))) = {{\n{body}\n}};\n'


def write_header(z, path):
    K, OD = int(z['K']), int(z['OD']); n_in = (K + 1) * OD
    W0, W1, W2 = (z[k].T.copy() for k in ('W0', 'W2', 'W4'))              # -> [out][in]
    mean = np.tile(z['mean'], K + 1); inv_std = 1.0 / np.tile(z['std'], K + 1)
    h = ['/* generated by deploy/export/export_actor.py -- do not edit */',
         '#pragma once',
         f'#define OASIS_K {K}', f'#define OASIS_OD {OD}', f'#define OASIS_N_IN {n_in}',
         f'#define OASIS_H1 {W0.shape[0]}', f'#define OASIS_H2 {W1.shape[0]}', f'#define OASIS_NOUT {W2.shape[0]}',
         f'#define OASIS_RESIDUAL {int(bool(z["residual"]))}', f'#define OASIS_RES_SCALE {c_float(z["res_scale"])}',
         f'#define OASIS_FF_GAIN {c_float(z["ff_gain"])}', f'#define OASIS_FF_IDX {int(z["ff_idx"])}',
         f'#define OASIS_PITCH_PRIOR {c_float(float(z["pitch_prior"]) if "pitch_prior" in z.files else 0.0)}',
         f'#define OASIS_FF_BOTH {c_float(float(z["ff_both"]) if "ff_both" in z.files else 0.0)}',
         f'#define OASIS_FF_DEADBAND {c_float(float(z["ff_deadband"]) if "ff_deadband" in z.files else 0.0)}', '']
    h += [c_array('OASIS_MEAN', mean), c_array('OASIS_INV_STD', inv_std),
          c_array('OASIS_W0', W0), c_array('OASIS_B0', z['b0']),
          c_array('OASIS_W1', W1), c_array('OASIS_B1', z['b2']),
          c_array('OASIS_W2', W2), c_array('OASIS_B2', z['b4'])]
    open(path, 'w').write('\n'.join(h))


def _pb():
    """ONNX protobuf classes. Uses the onnx package when it imports; otherwise loads its generated
    onnx_ml_pb2.py standalone (this PC's onnx wheel installs without onnx/defs)."""
    try:
        import onnx
        return onnx.onnx_ml_pb2
    except Exception:
        import importlib.util
        d = importlib.util.find_spec('onnx').submodule_search_locations[0]
        spec = importlib.util.spec_from_file_location('_onnx_ml_pb2', os.path.join(d, 'onnx_ml_pb2.py'))
        m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


def build_core_onnx(Ws, bs, conv):
    """3-layer MLP core as an ONNX ModelProto: Gemm/Relu (conv=False) or 1x1 Conv/Relu (conv=True)."""
    P = _pb(); F = P.TensorProto.FLOAT
    g = P.GraphProto(name='oasis_actor_core')

    def tensor(name, a):
        a = np.ascontiguousarray(a, np.float32); t = g.initializer.add()
        t.name, t.data_type = name, F; t.dims.extend(a.shape); t.raw_data = a.tobytes()

    def vinfo(coll, name, shape):
        v = coll.add(); v.name = name; v.type.tensor_type.elem_type = F
        for d in shape: v.type.tensor_type.shape.dim.add().dim_value = d

    n_in = Ws[0].shape[0]
    vinfo(g.input, 'obs_norm', (1, n_in, 1, 1) if conv else (1, n_in))
    x = 'obs_norm'
    for j, (W, b) in enumerate(zip(Ws, bs)):
        tensor(f'W{j}', W.T[:, :, None, None] if conv else W); tensor(f'b{j}', b)
        y = f'fc{j}' if j < 2 else 'core_out'
        n = g.node.add(); n.op_type = 'Conv' if conv else 'Gemm'; n.name = f'{n.op_type.lower()}{j}'
        n.input.extend([x, f'W{j}', f'b{j}']); n.output.append(y)
        if conv:
            a = n.attribute.add(); a.name, a.type = 'kernel_shape', P.AttributeProto.INTS; a.ints.extend([1, 1])
        if j < 2:
            r = g.node.add(); r.op_type = 'Relu'; r.name = f'relu{j}'; r.input.append(y); r.output.append(y + '_r'); y += '_r'
        x = y
    vinfo(g.output, 'core_out', (1, Ws[-1].shape[1], 1, 1) if conv else (1, Ws[-1].shape[1]))
    m = P.ModelProto(ir_version=8, producer_name='oasis-export', graph=g)
    o = m.opset_import.add(); o.domain = ''; o.version = 13
    return m


def write_onnx(z, out_dir):
    """Write both ONNX variants of the core and check them with onnxruntime.
    Returns {file: max abs error vs the numpy core}."""
    n_in = (int(z['K']) + 1) * int(z['OD'])
    Ws = [z['W0'], z['W2'], z['W4']]; bs = [z['b0'], z['b2'], z['b4']]
    rng = np.random.default_rng(0); zs = np.clip(rng.normal(0, 1.5, (256, n_in)), -5, 5).astype(np.float32)
    ref = zs
    for j, (W, b) in enumerate(zip(Ws, bs)):
        ref = ref @ W + b
        if j < 2: ref = np.maximum(ref, 0)
    errs = {}
    for name, conv in (('actor_core_fc.onnx', False), ('actor_core_conv.onnx', True)):
        p = os.path.join(out_dir, name)
        open(p, 'wb').write(build_core_onnx(Ws, bs, conv).SerializeToString())
        shape = (1, n_in, 1, 1) if conv else (1, n_in)
        try:
            import onnxruntime as ort
        except ImportError:
            errs[name] = None; continue
        s = ort.InferenceSession(p, providers=['CPUExecutionProvider'])
        got = np.concatenate([s.run(None, {'obs_norm': x.reshape(shape)})[0].reshape(1, -1) for x in zs])
        errs[name] = float(np.abs(got - ref).max())
    return errs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz'); ap.add_argument('--out', default=os.path.join(ROOT, 'deploy', 'model'))
    ap.add_argument('--no-onnx', action='store_true')
    ap.add_argument('--calib', help='calib json the model was trained with (default sim/v2/calib_v2.json)')
    ap.add_argument('--overlay', help='DR overlay(s) the model was trained with, os.pathsep-separated (pipeline: models/<v>/overlay.yaml + train overlays)')
    a = ap.parse_args()
    z = np.load(a.npz); os.makedirs(a.out, exist_ok=True)
    cpath, C = contract(int(z['OD']))
    for f in ('actor_weights.h', 'actor_core_fc.onnx', 'actor_core_conv.onnx', 'check_set.npy', 'calib_raw.npy', 'predictor.npz',
              'calib_norm_fc.npy', 'calib_norm_nhwc.npy', 'calib_actions.npy', 'quant_check.json'):
        if os.path.exists(os.path.join(a.out, f)): os.remove(os.path.join(a.out, f))      # never mix two models
    shutil.copyfile(a.npz, os.path.join(a.out, 'actor.npz'))
    shutil.copyfile(cpath, os.path.join(a.out, 'obs_layout.py'))
    write_header(z, os.path.join(a.out, 'actor_weights.h'))
    onnx_err = {} if a.no_onnx else write_onnx(z, a.out)
    K, OD = int(z['K']), int(z['OD'])
    defaults = model_defaults(OD, a.calib, a.overlay); pr = defaults.get('predictor') or {}
    if pr.get('npz_source'):                      # learned predictor ships with the model (sha in files)
        src = pr.pop('npz_source'); src = src if os.path.isabs(src) else os.path.join(ROOT, src)
        shutil.copyfile(src, os.path.join(a.out, 'predictor.npz')); pr['npz'] = 'predictor.npz'
    man = dict(source=os.path.relpath(os.path.abspath(a.npz), ROOT).replace('\\', '/'), source_sha256=sha(a.npz),
               obs_version=C.VERSION, obs_contract=os.path.relpath(cpath, ROOT).replace(os.sep, '/'),
               action_dim=int(z['W4'].shape[1]) // 2, **defaults, K=K, OD=OD, n_in=(K + 1) * OD,
               hidden=[int(z['W0'].shape[1]), int(z['W2'].shape[1])], n_out_core=int(z['W4'].shape[1]),
               macs=int(z['W0'].size + z['W2'].size + z['W4'].size),
               residual=bool(z['residual']), res_scale=float(z['res_scale']), ff_gain=float(z['ff_gain']), ff_idx=int(z['ff_idx']),
               pitch_prior=float(z['pitch_prior']) if 'pitch_prior' in z.files else 0.0,
               ff_both=float(z['ff_both']) if 'ff_both' in z.files else 0.0, ff_deadband=float(z['ff_deadband']) if 'ff_deadband' in z.files else 0.0,
               onnx_max_abs_err=onnx_err,
               files={f: sha(os.path.join(a.out, f)) for f in sorted(os.listdir(a.out)) if f != 'manifest.json'})
    json.dump(man, open(os.path.join(a.out, 'manifest.json'), 'w'), indent=1)
    print(json.dumps({k: v for k, v in man.items() if k != 'files'}, indent=1))


if __name__ == '__main__':
    main()
