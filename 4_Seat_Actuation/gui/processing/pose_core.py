"""Offline seated-person pose helpers. Full-image pixels; x right, y down."""
from pathlib import Path
import cv2
import numpy as np

NAMES=('nose','left_eye','right_eye','left_ear','right_ear','left_shoulder','right_shoulder',
       'left_elbow','right_elbow','left_wrist','right_wrist','left_hip','right_hip',
       'left_knee','right_knee','left_ankle','right_ankle')
BONES=((0,1),(0,2),(1,3),(2,4),(5,6),(5,7),(7,9),(6,8),(8,10),(5,11),(6,12),(11,12))
ROI=(300,120,920,720)
THRESHOLD=.5


def load_model(path,threads=2,backend='onnxruntime'):
    import onnxruntime as ort
    from rtmlib import RTMPose
    # RTMLib preprocessing/decoder, with an explicitly bounded CPU session.
    model=RTMPose.__new__(RTMPose)
    model.onnx_model=str(Path(path))
    model.model_input_size=(192,256)
    model.mean=(123.675,116.28,103.53)
    model.std=(58.395,57.12,57.375)
    model.backend=backend;model.device='cpu';model.to_openpose=False
    if backend=='openvino':
        from openvino import Core
        core=Core()
        net=core.read_model(str(path))
        model.compiled_model=core.compile_model(net,'CPU',{'PERFORMANCE_HINT':'LATENCY',
            'INFERENCE_NUM_THREADS':threads,'NUM_STREAMS':1,'INFERENCE_PRECISION_HINT':'f32'})
        model.input_layer=model.compiled_model.input(0)
        model._ov_outputs=[model.compiled_model.output(i) for i in range(len(net.outputs))]
        return model
    options=ort.SessionOptions()
    options.intra_op_num_threads=threads;options.inter_op_num_threads=1
    options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    providers=['CPUExecutionProvider']
    if backend=='directml':
        if 'DmlExecutionProvider' not in ort.get_available_providers():
            raise RuntimeError('Install onnxruntime-directml in a separate environment')
        model.backend='onnxruntime'
        options.enable_mem_pattern=False
        options.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        providers=['DmlExecutionProvider','CPUExecutionProvider']
    model.inference_backend=backend
    model.session=ort.InferenceSession(str(path),sess_options=options,providers=providers)
    return model


def estimate(model,frame,roi=ROI):
    x0,y0,x1,y1=roi
    xy,score=model(frame[y0:y1,x0:x1])
    xy=xy[0].astype(np.float32)+np.array([x0,y0],np.float32)
    score=score[0].astype(np.float32)
    valid=np.isfinite(xy).all(axis=1)&(score>=THRESHOLD)&(xy[:,0]>=x0)&(xy[:,0]<x1)&(xy[:,1]>=y0)&(xy[:,1]<y1)
    # These recordings end at the lap; knees/ankles are not observed.
    valid[13:]=False
    target=bool(valid[[0,5,6]].all() and 140<=xy[0,1]<=460 and
                abs(float(xy[0,0]-(xy[5,0]+xy[6,0])/2))<140 and
                (xy[5,1]+xy[6,1])/2>xy[0,1]+20)
    valid &= target
    return xy,score,valid,target


def features(xy,valid):
    result=dict(shoulder_tilt_deg=np.nan,torso_lean_deg=np.nan,head_offset_shoulder_ratio=np.nan,
                shoulder_span_px=np.nan,torso_height_px=np.nan)
    if valid[[5,6]].all():
        a,b=xy[5],xy[6]
        if a[0]>b[0]:a,b=b,a
        span=float(np.linalg.norm(b-a))
        shoulder=(a+b)/2
        result['shoulder_tilt_deg']=float(np.degrees(np.arctan2(b[1]-a[1],b[0]-a[0])))
        result['shoulder_span_px']=span
        if valid[0] and span>1:
            result['head_offset_shoulder_ratio']=float((xy[0,0]-shoulder[0])/span)
        if valid[[11,12]].all():
            hip=(xy[11]+xy[12])/2
            height=float(hip[1]-shoulder[1])
            if height>30:
                result['torso_lean_deg']=float(np.degrees(np.arctan2(shoulder[0]-hip[0],height)))
                result['torso_height_px']=height
    return result


def draw(frame,xy,score,valid,roi=ROI,labels=()):
    x0,y0,x1,y1=roi
    image=frame[y0:y1,x0:x1].copy()
    points=np.rint(xy-[x0,y0]).astype(np.int32)
    for a,b in BONES:
        if valid[a] and valid[b]:cv2.line(image,tuple(points[a]),tuple(points[b]),(30,220,255),3,cv2.LINE_AA)
    for i in range(13):
        if valid[i]:cv2.circle(image,tuple(points[i]),4,(40,240,40),-1,cv2.LINE_AA)
    for i,text in enumerate(labels):
        cv2.putText(image,str(text),(8,22+22*i),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(image,str(text),(8,22+22*i),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1,cv2.LINE_AA)
    return image
