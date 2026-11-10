# Session 4 검증 (2026-11-10)

macOS 27.2 · Apple Silicon · Python 3.10.21 · 기존 소스 빌드 ROS 2 Jazzy · Gazebo Sim 8.15에서 검사했습니다. 실물 센서·차량을 사용하지 않았습니다.

## 확인한 항목

| 항목 | 결과 |
|---|---|
| 기존 형식 | `src/session_4`, `docs/SESSION_4.md`, `scripts/session_4*.sh`, ament Python 패키지 |
| 빌드·폴더 구조 | `student/driving.py` 하나와 `common/`으로 재배치, colcon 빌드·공유 자산·학생 파일 연결 확인 |
| 환경 스크립트 | bash·zsh에서 도메인 40 및 localhost 통신 설정 확인 |
| 맵 자산 | SDF 검사 Valid, XML 파싱·메시 법선 검사, 외부 Fuel 다운로드 없이 설치 자산 로딩 |
| 알고리즘·맵·도구 검사 | 현재 Session 4 검사 47개 통과 |
| 실제 Gazebo 센서 | 320×240 RGB 카메라 2개·CameraInfo·360방향 LiDAR·clock 수신 |
| 실제 Gazebo 물리 이동 | 직진 0.3091m, 양수 조향에서 좌회전 +0.0832rad |
| 명시 정지·명령 미수신 정지 | 정지 후 위치 변화 각각 0m 및 약 8.1×10⁻¹⁰m |
| 실제 Gazebo 신호 변경 | 카메라 영상에서 빨강 → 초록을 확인; 색 예제 검출기가 실제 영상에서 red/green 검출 |
| 정답코드 라바콘·신호 | 기존 라바콘 배열을 신호등 반대편 직선으로 180도 회전 배치. 변경된 맵에서 라바콘 주행·빨강 정지·초록 재출발·한 바퀴 완주 확인 |
| 학생 파일 직접 실행 | 환경을 불러오지 않은 `python3 student/driving.py --help` 실행 확인. 실제 주행 검사는 학생 파일을 `--solution`으로 직접 실행 |
| 정답코드 한 바퀴 | 단일 노드·함수 반환값 구조와 제공 YOLO로 한 바퀴 통과. 오른쪽 차로 중심 최대 오차 0.1706m, 종료 후 위치 변화 0m |
| 역할 연결 | `lane(camera)`, `rubbercone(lidar)`, `yolo(labels,bbox,size,conf)`의 반환값을 `step(lane_result,rubbercone_result,yolo_result)`에 전달. 역할별 토픽·프로세스 제거 |
| 입력 만료 | 센서 미수신 시 해당 함수 결과를 None으로 전달하고 정지. 만료 경계와 제어 입력별 정지는 자동 검사로 확인 |
| 수동 운전 창 | offscreen Qt 키 입력 검사: 키 누름·해제, Space, 창 이탈·종료 시 정지 확인. 실제 저장 카메라 영상으로 화면 검수 |
| 수동·자동 모드 | 실제 ROS에서 선택하지 않은 명령 무시, 전환 시 기존 명령 초기화, 수동 명령 중단 후 0.5161초에 정지 |
| 녹화·사진 추출 | 실제 녹화 도구로 높은/낮은 카메라 각 20장 저장, 정상 Ctrl+C 종료·완료 표시, 5프레임 간격 사진 4장 추출·원본 bag 보존 |
| 제공 YOLO | Gazebo 영상 train 210장·val 59장, HSV 자동 라벨, MPS 40 epoch 학습. 가중치·해시·학습 정보 포함. 실제 Gazebo에서 추론·빨강 정지·초록 출발·라바콘 모드 확인 |
| 정답 시각화 | 정답 실행 시 자동 생성, 카메라·라이다·주행 설명 표시. 창 종료 시 주행 프로세스 종료 및 정지 확인 |

자율주행 코드(`solution.py`)는 카메라·LiDAR·검출 결과만 읽습니다. 검사기의 Gazebo world pose는 이동·차로 이탈 평가에만 사용합니다. `/session4/traffic_light/state`를 주행 판단에 사용하지 않습니다.

## 증거와 재현

검사 결과와 센서 PNG는 Git에서 제외되는 `workspace/session_4/validation/`에 보존했습니다.

- [센서·물리 이동·정지 결과](../../../workspace/session_4/validation/sim_runtime__apak1w3/result.json)
- [이전 역할별 토픽 구조의 미수신 결과](../../../workspace/session_4/validation/roles_runtime_nrj2w9a_/result.json)
- [이전 구조의 색 검출기 ROS 결과](../../../workspace/session_4/validation/roles_runtime_26tjjbus/result.json)
- [단일 학생 파일로 한 바퀴 주행](../../../workspace/session_4/validation/driving_runtime_femv0rde/result.json), [전체 주행 추적](../../../workspace/session_4/validation/driving_runtime_femv0rde/trace.json)
- [빨간불 정지 영상](../../../workspace/session_4/validation/driving_runtime_femv0rde/red_stop_high.png), [초록불 재출발 영상](../../../workspace/session_4/validation/driving_runtime_femv0rde/green_restart_high.png)
- [수동·자동 선택과 녹화·추출](../../../workspace/session_4/validation/classroom_runtime_0y8prpel/result.json)
- [수동 운전 창 검수](../../../workspace/session_4/validation/manual_window.png), [디버그 화면 검수](../../../workspace/session_4/validation/debug_viewer_layout.png)
- [실제 YOLO 학습·결과 재로딩](../../../workspace/session_4/validation/yolo_training_smoke_f8oc5zwo/result.json)

```bash
source scripts/session_4_env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/session_4/common/test -q
python src/session_4/common/tools/check_sim_runtime.py
python src/session_4/common/tools/check_driving_runtime.py --lap
python src/session_4/common/tools/check_classroom_runtime.py
```

검사 도메인은 센서/물리 83, 주행 84, 녹화·모드 선택 87로 나눕니다. 각 검사는 직접 만든 프로세스를 종료하고 회수합니다. 녹화 검사·학습 smoke는 운영용 최신 결과 표시를 바꾸지 않도록 복원하거나 별도 프로젝트에서 실행했습니다. Gazebo 렌더링과 로컬 통신을 허용해야 합니다.

`bash scripts/session_4.sh sim`과 `build`는 패키지를 빌드합니다. 설치가 준비된 뒤 `manual`·`collect` 등은 재빌드하지 않습니다. 설치된 모델·메시·월드 링크는 임시 `build/session_4/common`을 거치지 않고 원본 자산을 직접 가리키도록 수정했습니다.

수동주행 카메라에서 트랙이 안 보인다는 제보를 같은 `sim → manual` 명령으로 확인했으나, 수정 전 검사에서는 원본 영상에 도로가 정상 표시되어 현상을 재현하지 못했습니다. 빌드 중 모델 경로가 바뀌는 가능성을 제거한 뒤, Gazebo GUI와 수동주행을 함께 실행해 높은/낮은 카메라 각각 51프레임 수신 및 도로·차선 표시를 확인했습니다. [수정 후 검사 결과](../../../workspace/session_4/validation/manual_render_after/result.json), [수동주행 카메라 원본](../../../workspace/session_4/validation/manual_render_after/high.png). 따라서 Metal 자체의 렌더링 결함으로 확정한 것은 아닙니다.

초기 색 검출기가 곡선/직선의 노란 점선을 신호로 잘못 읽은 현상을 실제 주행에서 발견했습니다. 제공 카메라의 신호등 검출 영역을 화면 위 절반으로 좁혔고, 실제 저장 영상에서 노란 점선은 `unknown`, 정지 신호는 `red`, 출발 신호는 `green`으로 확인했습니다. 실패한 초기 실행 기록도 삭제하지 않았습니다.

## 확인하지 않은 범위

Ubuntu 24.04·Windows WSL2의 실행, 빈 노트북의 전체 설치, 실제 주행 사진으로 학습한 YOLO의 정확도·추론 속도는 별도 확인이 필요합니다. 초기 smoke 검사는 합성 데이터로 학습 파이프라인만 확인했습니다. 현재 제공 모델의 자동 라벨 검증 mAP50은 0.9813이나, 독립적인 사람이 붙인 정답이나 실제 도로 정확도를 의미하지 않습니다. 상세 정보는 `weights/training_info.json`에 있습니다. Gazebo 3D GUI 프로세스의 시작·정상 종료는 확인했지만, 네이티브 창의 화면 검수는 수행하지 못했습니다. 차량 카메라에서 렌더링된 도로·차선·라바콘·신호는 실제 PNG로 확인했습니다.

기본 색 검출기와 카메라 역투영 수치는 제공된 Gazebo 환경의 교육 예제입니다. 다른 조명·맵·카메라나 실제 차량에 그대로 일반화된다는 의미는 아닙니다.

구현 근거: [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/), [공식 ROS–Gazebo bridge](https://github.com/gazebosim/ros_gz), [Gazebo UserCommands의 visual_config](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/user_commands/UserCommands.cc), [Ultralytics 추론 API](https://docs.ultralytics.com/modes/predict/). 수업 흐름과 복사할 명령은 [학생용 안내](../../../docs/SESSION_4.md)에 있습니다.
