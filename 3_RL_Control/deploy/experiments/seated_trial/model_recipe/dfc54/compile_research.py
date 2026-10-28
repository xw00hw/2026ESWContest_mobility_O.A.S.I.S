import hashlib
import json
import os
from pathlib import Path
import resource
import time
import traceback
OUT=Path(os.environ['OASIS_DFC_RUN_DIR'])
receipt=Path(os.environ['OASIS_DFC_QUANTIZATION_RESULT'])
start=time.monotonic()
report=dict(stage='compilation',hw_arch='hailo10h',purpose='historical_seated_research_candidate',
            deployment_approved=False,current_ov9281_representativeness_verified=False,
            camera_motor_control_enabled=False,compilation_success=False)
try:
    quant=json.loads(receipt.read_text())
    if not quant.get('quantization_success') or not quant.get('heldout_complete'):
        raise RuntimeError('Quantization/heldout result is incomplete')
    audit_path=receipt.parent/'heldout_confidence_audit.json'
    confidence_pass=False
    if audit_path.is_file():
        audit=json.loads(audit_path.read_text())
        if audit['quantization_receipt_sha256']!=hashlib.sha256(receipt.read_bytes()).hexdigest():raise RuntimeError('Confidence audit receipt mismatch')
        confidence_pass=audit['all_reference_confident_core_joints_retained'] is True
        report['confidence_audit_sha256']=hashlib.sha256(audit_path.read_bytes()).hexdigest()
    report.update(research_quality_pass=bool(quant.get('research_quality_pass',False) and confidence_pass),
                  all_reference_confident_core_joints_retained=confidence_pass,
                  benchmark_only=True,pose_control_usable=False,
                  heldout_prefix_snr_db=quant.get('prefix_snr_db'),
                  heldout_core_max_shift_input_pixels=quant.get('heldout_confident_core_max_input_pixel_shift'),
                  source_optimized_har_sha256=quant['optimized_har_sha256'],
                  quantization_result_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
    source=Path(quant['optimized_har'])
    if hashlib.sha256(source.read_bytes()).hexdigest()!=quant['optimized_har_sha256']:
        raise RuntimeError('Optimized HAR hash mismatch')
    from hailo_sdk_client import ClientRunner
    runner=ClientRunner(har=str(source),hw_arch='hailo10h')
    boundary=quant.get('boundary_contract',{}).get('boundary','Conv_205:625')
    if boundary not in ('Conv_205:625','Mul_204:624'):raise RuntimeError('Unexpected boundary')
    report['boundary']=boundary
    allocation_export=os.environ.get('OASIS_DFC_ALLOCATION_EXPORT')
    if allocation_export:
        import numpy as np
        ep=Path(allocation_export);export=json.loads(ep.read_text())
        if export['public_api']!='ClientRunner.save_autogen_allocation_script' or not export['created'] or export['boundary']!=boundary:
            raise RuntimeError('Wrong exported allocation provenance')
        original_compile_path=ep.parent/'compilation_result.json'
        if hashlib.sha256(original_compile_path.read_bytes()).hexdigest()!=export['source_compilation_receipt_sha256']:
            raise RuntimeError('Allocation source compilation receipt mismatch')
        original_compile=json.loads(original_compile_path.read_text())
        original_quant_path=Path(original_compile['quantization_result'])
        if not original_compile['compilation_success'] or original_compile['hef_sha256']!=export['source_hef_sha256'] or hashlib.sha256(original_quant_path.read_bytes()).hexdigest()!=original_compile['quantization_result_sha256']:
            raise RuntimeError('Allocation source model lineage mismatch')
        original_quant=json.loads(original_quant_path.read_text())
        if original_quant['source_har_sha256']!=quant['source_har_sha256'] or original_quant.get('boundary_contract')!=quant.get('boundary_contract'):
            raise RuntimeError('Allocation source is a different parsed model or boundary')
        script_path=Path(export['script_path']);script_sha=hashlib.sha256(script_path.read_bytes()).hexdigest()
        if script_sha!=export['script_sha256']:raise RuntimeError('Exported allocation script hash mismatch')
        def fingerprint():
            h=hashlib.sha256()
            for name,value in sorted(runner.get_params_translated().items()):
                arr=np.asarray(value)
                data=json.dumps(arr.tolist(),sort_keys=True,default=str).encode() if arr.dtype.hasobject else np.ascontiguousarray(arr).tobytes()
                meta=json.dumps([name,str(arr.dtype),list(arr.shape),len(data)]).encode()
                h.update(len(meta).to_bytes(8,'little'));h.update(meta);h.update(data)
            hn_hash=hashlib.sha256(json.dumps(runner.get_hn_dict(),sort_keys=True,separators=(',',':'),default=str).encode()).hexdigest()
            return {'hn_sha256':hn_hash,'translated_parameters_sha256':h.hexdigest(),'state':str(runner.state)}
        before=fingerprint()
        runner.load_model_script(str(script_path))
        after=fingerprint()
        report['allocation_reuse']={'export_receipt_path':str(ep),'export_receipt_sha256':hashlib.sha256(ep.read_bytes()).hexdigest(),
            'script_sha256':script_sha,'source_hef_sha256':export['source_hef_sha256'],'before':before,'after':after,
            'public_api':'ClientRunner.load_model_script','quantized_graph_and_parameters_unchanged':before==after}
        (OUT/'allocation_reuse_check.json').write_text(json.dumps(report['allocation_reuse'],indent=2)+'\n')
        if before!=after:raise RuntimeError('Allocation script changed validated quantized graph or parameters')
        (OUT/'reused_autogenerated_allocation.alls').write_bytes(script_path.read_bytes())
    hef=runner.compile()
    stem='rtmpose_'+boundary.split(':')[0].lower()
    path=OUT/(stem+'_hailo10h_research.hef')
    with path.open('xb') as f:f.write(hef)
    runner.save_har(str(OUT/(stem+'_research_compiled_model.har')))
    report.update(compilation_success=True,hef_path=str(path),hef_bytes=path.stat().st_size,
                  hef_sha256=hashlib.sha256(hef).hexdigest(),quantization_result=str(receipt),boundary=boundary)
except BaseException as exc:
    report.update(error_type=type(exc).__name__,error=str(exc));traceback.print_exc()
finally:
    report.update(elapsed_s=time.monotonic()-start,max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    (OUT/'compilation_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['compilation_success'] else 1)
