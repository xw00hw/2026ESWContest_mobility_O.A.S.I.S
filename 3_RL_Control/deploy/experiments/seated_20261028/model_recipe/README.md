# RTMPose Hailo 부분 가속 연구 레시피

2026-10-28 실기 파이프라인에 이어진 변환·검증 코드의 **소스 참조본**이다. 범용 설치기나 즉시 실행 가능한 배포 명령이 아니다. 각 스크립트의 기존 절대경로, 모델 경계, 해시 고정 조건을 보존했다. 실행 전에 별도 연구 작업공간에서 경로와 의존성을 검토해야 한다.

- `model_transform/split_before_final_conv.py`: 원본 RTMPose의 `Mul_204:624` 경계로 분리한다. Hailo 앞부분과 CPU 마지막 합성곱/SimCC head를 위한 연구 코드다.
- `model_transform/validate_hybrid_hardware.py`: 기본값은 모델/배열/해시 검증만 수행한다. `--run-device`가 명시되면 Hailo 장치를 사용한다. 현재 카메라를 열거나 모터를 제어하지 않는다.
- `dfc54/`: 당시 DFC 5.4 작업공간(`/root/...`)에서 사용했던 검증, 컴파일, mapping 추출, manifest 준비 코드다. 고정 경로와 외부 HAR/ONNX/보정·검증 corpus가 필요하다. import 또는 실행만으로 변환을 시작하는 파일이 있으므로 라이브러리처럼 import하지 않는다.

모델 원본/가중치, ONNX/HAR/HEF 바이너리, 보정 및 held-out 배열, DFC 설치 파일, HailoRT·드라이버 설치기는 포함하지 않는다. 이 폴더를 보존하며 컴파일이나 장치 추론을 다시 실행하지 않았다. 단위시험의 mock 결과와 실제 Hailo 품질 검증은 구별한다. 원본과 해시 일치는 상위 `source_provenance.json`에 기록한다.

현재 실사용 방식은 Hailo-10H 앞부분 + CPU 뒷부분의 혼합 추론이다. 전체 RTMPose를 Hailo에서 실행한다고 표현하지 않는다. 분리 부동소수점의 수치 일치나 합성 진단만으로 사람 자세 정확도 또는 착석 효과가 검증됐다고 주장하지 않는다.
