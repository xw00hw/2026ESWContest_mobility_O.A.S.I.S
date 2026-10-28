"""Exact earlier original RTMPose split: Mul_204:624 | Conv_205 + SimCC head.

CPU-only derivation and float parity. Preserves original and existing artifacts;
no DFC optimization, model weights modification, runtime edit or device access.
"""
from pathlib import Path
from datetime import datetime,timezone
import copy,hashlib,json,time
import numpy as np
import onnx
from onnx import shape_inference,utils
import onnxruntime as ort

HERE=Path(__file__).resolve().parent
ROOT=Path('/root/ESW_RTMPose_Compile')
ORIGINAL=ROOT/'models/rtmpose_m.onnx'
ORIGINAL_SHA='5c0a4bf67953e6d2ac43ce15e77dc9d5d354ae18430a47d2c5963a7bc5683e3c'
digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
if digest(ORIGINAL)!=ORIGINAL_SHA:raise ValueError('source_identity_mismatch')
out=HERE/('derived_mul204_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
out.mkdir()
model=onnx.load(ORIGINAL)
nodes={n.name:n for n in model.graph.node}
if list(nodes['Mul_204'].output)!=['624'] or nodes['Conv_205'].input[0]!='624':
    raise ValueError('source_boundary_mismatch')
fixed=copy.deepcopy(model)
dim=fixed.graph.input[0].type.tensor_type.shape.dim[0]
dim.ClearField('dim_param');dim.dim_value=1
fixed=shape_inference.infer_shapes(fixed,strict_mode=True)
extractor=utils.Extractor(fixed)
prefix=extractor.extract_model(['input'],['624'])
head=extractor.extract_model(['624'],['simcc_x','simcc_y'])
for derived in (prefix,head):onnx.checker.check_model(derived)
prefix_path=out/'rtmpose_m_mul204_prefix.onnx'
head_path=out/'rtmpose_m_conv205_simcc_head.onnx'
onnx.save(prefix,prefix_path);onnx.save(head,head_path)
opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
session=lambda path:ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
ref,pref,tail=map(session,(ORIGINAL,prefix_path,head_path))
assert pref.get_outputs()[0].shape==[1,768,8,6]
assert tail.get_inputs()[0].name=='624' and tail.get_inputs()[0].shape==[1,768,8,6]
archive=json.loads((HERE/'parity_inputs.json').read_text())
if digest(HERE/'parity_inputs.npz')!=archive['archive_sha256']:raise ValueError('parity_input_archive_mismatch')
inputs=np.load(HERE/'parity_inputs.npz',allow_pickle=False)['input']
corpus_manifest=ROOT/'research_calibration/manifest.json'
corpus=json.loads(corpus_manifest.read_text())
heldout_path=ROOT/'research_calibration/holdout.npy'
if digest(heldout_path)!=corpus['tensors']['holdout']['sha256']:raise ValueError('heldout_hash_mismatch')
heldout=np.load(heldout_path,allow_pickle=False)
assert heldout.shape==(32,256,192,3) and heldout.dtype==np.float32
all_inputs=[(f'parity_{i}',archive['samples'][i]['kind'],x[None]) for i,x in enumerate(inputs)]
all_inputs += [(f'heldout_{i}','historical_seated_holdout',x[None].transpose(0,3,1,2)) for i,x in enumerate(heldout)]
rows=[];heldout_features=[]
for sample,kind,value in all_inputs:
    x=np.ascontiguousarray(value)
    expected=ref.run(['simcc_x','simcc_y'],{'input':x})
    feature=pref.run(['624'],{'input':x})[0]
    started=time.perf_counter_ns()
    actual=tail.run(['simcc_x','simcc_y'],{'624':feature})
    latency=(time.perf_counter_ns()-started)/1e6
    maxerror=max(float(abs(a-b).max()) for a,b in zip(actual,expected))
    changes=[]
    for axis,(a,b) in enumerate(zip(actual,expected)):
        np.testing.assert_allclose(a,b,rtol=1e-5,atol=1e-5)
        old,new=b.argmax(axis=-1)[0],a.argmax(axis=-1)[0]
        for joint in np.flatnonzero(old!=new):
            gap=float(b[0,joint,old[joint]]-b[0,joint,new[joint]])
            confidence=float((expected[0][0,joint].max()+expected[1][0,joint].max())*.5)
            if gap>2*maxerror+1e-7:raise ValueError('non_tie_argmax_change')
            if joint in (5,6,11,12) and confidence>=.5:raise ValueError('confident_core_argmax_change')
            changes.append(dict(axis=axis,joint=int(joint),reference_logit_gap=gap,confidence=confidence))
    rows.append(dict(sample=sample,kind=kind,max_simcc_abs_error=maxerror,
                     near_tie_argmax_changes=changes,confident_core_argmax_changes=0,wsl_cpu_head_latency_ms=latency))
    if kind=='historical_seated_holdout':heldout_features.append(feature.transpose(0,2,3,1)[0])
features_path=out/'heldout_float_prefix_mul204_nhwc.npy'
np.save(features_path,np.ascontiguousarray(heldout_features,dtype=np.float32))
def metadata(path,s):
    return dict(path=str(path),sha256=digest(path),bytes=path.stat().st_size,
                inputs=[dict(name=v.name,shape=v.shape,type=v.type) for v in s.get_inputs()],
                outputs=[dict(name=v.name,shape=v.shape,type=v.type) for v in s.get_outputs()])
report=dict(schema='oasis.rtmpose.original_graph_split_parity.v1',result='pass',source_model_sha256=ORIGINAL_SHA,
            source_script_sha256=digest(__file__),boundary='Mul_204:624',cpu_head_starts_at='Conv_205',
            operation='move unchanged final7x7conv and unchangedScaleNorm/GAU/SimCC intoCPUhead',
            sample_count=len(rows),parity_input_archive_sha256=archive['archive_sha256'],
            historical_manifest_sha256=digest(corpus_manifest),historical_heldout_sha256=digest(heldout_path),
            artifacts=dict(prefix=metadata(prefix_path,pref),cpu_head=metadata(head_path,tail)),
            heldout_float_prefix=dict(path=str(features_path),sha256=digest(features_path),shape=[32,8,6,768],dtype='float32'),
            boundary_contract=dict(hailo_output_nhwc=[1,8,6,768],cpu_head_input_nchw=[1,768,8,6],cpu_head_input_name='624',
                                   host_float32_bytes_per_frame=8*6*768*4),
            max_simcc_abs_error=max(r['max_simcc_abs_error'] for r in rows),
            confident_core_argmax_changes=0,
            original_modified=False,quantization_performed=False,hardware_inference_performed=False,
            runtime_modified=False,deployment_approved=False,latency_is_wsl_cpu_not_pi=True,samples=rows)
if digest(ORIGINAL)!=ORIGINAL_SHA:raise ValueError('source_changed')
with (out/'parity_report.json').open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
