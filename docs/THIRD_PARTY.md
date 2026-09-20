# 출처와 변경 범위

## 대회 저장소

- 저장소: https://github.com/hello-osy/ai-autonomous-driving-competition-2026/tree/osy-260809
- 고정 커밋: `3a29efdae1467355c1d7f059f92238febf85a33a`
- 복사: `lane_seg/models/lane_pidnet_s.pt`, `lane_seg/models/dataset_info.json`.
- 재구성: `lane_seg/dsbuild/segmenter.py`의 전처리/추론. strict 가중치 검증, weights_only 로딩, CPU/MPS 지원, 설치된 package share 경로 사용을 추가.
- 참고: `lane_offset/lane_offset/timed_lane_offset_node.py`의 측정 x/기준 x 부호와 130px→45도 변환. 원본 BEV/근접 영역/초록 매트 검증/중앙선 fallback은 단일 scan line 수업에서 생략.
- 이식: `drive_control/drive_control/drive_control_node.py`의 PID 계산, A1→각도, 각속도 필터. P/I/D=6.5/0/0.8, ±150 PWM, 최소40, 허용오차1도, 적분 제한30PWM, alpha=.25.
- 차이: 원본은 별도 20Hz 타이머로 실제 아두이노 명령을 발행. 여기서는 카메라 처리 시각마다 교육용 계산/표시하며 모터 명령을 발행하지 않음. bag 반복 시 PID/그래프 상태 초기화. 별도 모의 피드백은 수업용 추가 기능.
- 원본 대회 저장소 루트에서 별도 라이선스 파일을 찾지 못했으므로 해당 코드/가중치에 임의의 오픈소스 라이선스를 부여하지 않음. 사용자 지정 자료로 교육 프로젝트 안에서 사용.

## PIDNet 공식 모델 정의

- https://github.com/XuJiacong/PIDNet
- 고정 커밋: `4c158cf24ce432f0a8cb43364fae38d93cee0dc3` (대회 저장소 setup_pidnet.sh와 동일)
- `models/pidnet.py`, `models/model_utils.py` 두 파일만 변경 없이 vendoring.
- MIT 라이선스 원문: `src/session_1/session_1/vendor/LICENSE`.
- 학습용 데이터/도구 및 600MB 전체 저장소를 가져오지 않음.

## 첨부 교육자료

- 최우선: `2026-2학기 ORDA 교육자료(260920).pptx` 4~12쪽의 인지/판단/제어 및 segmentation→scan line→PID 흐름.
- `01_차선주행전체과정.pdf`: 단계별 입력/출력과 오차 시각화 설명.
- `03_스켈레톤코드설명.pdf`, `03_skeleton_code.py`: 센서 수신/계산/출력 분리 참고. ROS1 API와 90 중심 servo 명령은 이 ROS2 PWM 구현에 복사하지 않음.
- 자료의 모든 미션/ROS1 설치 지시를 작업 요구로 해석하지 않고 요청된 session_1 범위만 구현.

## 포함 파일 SHA-256

- `models/lane_pidnet_s.pt`: `98ebafcb4f251a952f4bd71454fcafa52620d70702724f2edfb503e614413061`
- `models/dataset_info.json`: `701a0c7485cf8c2c1a5222a7938a56dfd2da80435895c6fd6bb0e8ef487c06a7`
- `session_1/vendor/pidnet.py`: `1692ebc05ea4cc7096abd0b9a2e7e79543d878955af1a9d3d60e7973d6eb623d`
- `session_1/vendor/model_utils.py`: `8817f6367b4c9d094b35cd606926f2937cef98469fff888e3773c1c95ca969aa`
- `session_1/vendor/LICENSE`: `e07b93980bc49ee3a99f61095208c426f592f54eaf501bf3f7ef5cd68120bd9b`
