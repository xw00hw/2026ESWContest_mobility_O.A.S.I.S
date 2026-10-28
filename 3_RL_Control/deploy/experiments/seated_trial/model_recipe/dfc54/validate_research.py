"""Fresh-process validation of a hash-pinned completed optimizer artifact; no optimization."""
import hashlib,json,os,resource,shutil,time,traceback
from pathlib import Path
import numpy as np
import onnxruntime as ort

ROOT=Path('/root/ESW_RTMPose_Compile')
OUT=Path(os.environ['OASIS_DFC_RUN_DIR']);source_receipt=Path(os.environ['OASIS_DFC_QUANTIZATION_RESULT'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
start=time.monotonic();report={'stage':'fresh_process_heldout_validation','heldout_complete':False}
try:
    q=json.loads(source_receipt.read_text());process=json.loads((source_receipt.parent/'process_result.json').read_text())
    if not q.get('quantization_success') or q.get('validation_pending') is not True or process['stage']!='quantize' or Path(process['output']).resolve()!=source_receipt.parent.resolve() or process['exit_code']!=0 or process['stopped'] is not None:
        raise RuntimeError('Source optimization did not complete successfully')
    report.update(q)
    report.update(stage='fresh_process_heldout_validation',heldout_complete=False,research_quality_pass=False,
                  original_optimization_receipt_path=str(source_receipt),original_optimization_receipt_sha256=sha(source_receipt),
                  optimizer_elapsed_s=q['elapsed_s'],optimizer_max_rss_kib=q['max_rss_kib'],optimization_process_result=process,
                  optimization_process_result_sha256=sha(source_receipt.parent/'process_result.json'))
    contract=q['boundary_contract'];shape=tuple(contract['output_shape_nhwc'])
    if contract['boundary']!='Mul_204:624' or shape!=(8,6,768) or contract['head_input_name']!='624' or contract['prefix_sha256']!='feb57245237d06b24450b59843beb8e7faf977c599b18c39c4ac0fc80b63e028' or contract['head_sha256']!='62815b915d90d26607282a86aa403652225aed8ee1447d22c0e48bcd5d493251':
        raise RuntimeError('Unregistered boundary contract')
    if q['hw_arch']!='hailo10h' or sha(Path(q['optimized_har']))!=q['optimized_har_sha256']:
        raise RuntimeError('Optimized HAR/target mismatch')
    for kind in ('prefix','head'):
        if sha(Path(contract[kind]))!=contract[kind+'_sha256']:raise RuntimeError(kind+' hash mismatch')
    manifest_path=ROOT/'research_calibration/manifest.json'
    if sha(manifest_path)!=q['calibration_manifest_sha256']:raise RuntimeError('Corpus manifest mismatch')
    corpus=json.loads(manifest_path.read_text());entry=corpus['tensors']['holdout'];input_path=Path(entry['path'])
    if sha(input_path)!=entry['sha256']:raise RuntimeError('Heldout input mismatch')
    data=np.load(input_path,allow_pickle=False)
    reference_path=Path(q['reference_prefix_float_path'])
    if reference_path.parent!=source_receipt.parent or sha(reference_path)!=q['reference_prefix_float_sha256']:
        raise RuntimeError('Original float reference mismatch')
    reference=np.load(reference_path,allow_pickle=False)
    if data.shape!=(32,256,192,3) or reference.shape!=(32,*shape) or data.dtype!=np.float32 or reference.dtype!=np.float32 or not np.isfinite(data).all() or not np.isfinite(reference).all():
        raise RuntimeError('Unexpected input/reference tensor')
    for name,key in [('model_script.alls','precheck_model_script_sha256'),('post_fp_model_script.alls','model_script_sha256'),('additional_precision_proposal.json','additional_precision_proposal_sha256')]:
        p=source_receipt.parent/name
        if sha(p)!=q[key]:raise RuntimeError('Optimizer script/proposal mismatch')
        shutil.copy2(p,OUT/name)
    from hailo_sdk_client import ClientRunner,InferenceContext
    runner=ClientRunner(har=q['optimized_har'],hw_arch='hailo10h')
    print('STAGE fresh_process_quantized_heldout_32',flush=True)
    with runner.infer_context(InferenceContext.SDK_QUANTIZED) as context:value=runner.infer(context,data,batch_size=1)
    if isinstance(value,dict):
        if len(value)!=1:raise RuntimeError('Unexpected output count')
        value=next(iter(value.values()))
    quant=np.asarray(value)
    if quant.shape==(1,32,*shape):quant=quant[0]
    if quant.shape!=(32,*shape) or quant.dtype!=np.float32 or not np.isfinite(quant).all():raise RuntimeError('Unexpected quantized output')
    np.save(OUT/'heldout_prefix_float_nhwc.npy',reference)
    np.save(OUT/'heldout_prefix_quantized_nhwc.npy',quant)
    report['expected_arrays_sha256']={k:sha(OUT/('heldout_prefix_'+k+'_nhwc.npy')) for k in ('float','quantized')}
    error=quant-reference
    report['prefix_snr_db']=float(10*np.log10(np.mean(reference.astype(np.float64)**2)/max(np.mean(error.astype(np.float64)**2),1e-30)))
    options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
    head=ort.InferenceSession(contract['head'],sess_options=options,providers=['CPUExecutionProvider'])
    rows=[];core_errors=[];core=np.array([5,6,11,12])
    for i,(ref,x) in enumerate(zip(reference,quant)):
        ref_out=head.run(None,{'624':np.ascontiguousarray(ref[None].transpose(0,3,1,2))})
        q_out=head.run(None,{'624':np.ascontiguousarray(x[None].transpose(0,3,1,2))})
        score=(ref_out[0].max(axis=2)+ref_out[1].max(axis=2))[0]*.5
        shifts=np.stack([np.abs(a.argmax(axis=2)-b.argmax(axis=2))[0]/2. for a,b in zip(ref_out,q_out)],axis=-1)
        good=score[core]>=.5;core_errors.extend(shifts[core][good].max(axis=1).tolist())
        rows.append(dict(index=i,confident_core_joints=int(good.sum()),
            confident_core_max_input_pixel_shift=float(shifts[core][good].max()) if good.any() else None,
            simcc_max_abs=float(max(np.max(np.abs(a-b)) for a,b in zip(ref_out,q_out)))))
    report['heldout']=rows;report['heldout_confident_core_joints']=sum(r['confident_core_joints'] for r in rows)
    report['heldout_confident_core_max_input_pixel_shift']=max(core_errors) if core_errors else None
    report['heldout_confident_core_shift_p50_input_pixels']=float(np.percentile(core_errors,50)) if core_errors else None
    report['heldout_confident_core_shift_p95_input_pixels']=float(np.percentile(core_errors,95)) if core_errors else None
    report['research_quality_criteria']={'prefix_snr_min_db':10.0,'confident_core_max_shift_input_pixels':5.0,'not_physical_accuracy_validation':True}
    report['research_quality_pass']=bool(report['prefix_snr_db']>=10.0 and core_errors and max(core_errors)<=5.0)
    report.update(heldout_complete=True,validation_pending=False)
except BaseException as exc:
    report.update(error_type=type(exc).__name__,error=str(exc));traceback.print_exc()
finally:
    report.update(elapsed_s=time.monotonic()-start,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    (OUT/'quantization_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report.get('heldout_complete') else 1)
