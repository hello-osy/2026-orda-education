# 상세 사용법 · session_1

초보자용 통합 창은 `bash start.sh`로 실행합니다. 아래는 ROS 토픽 통신을 따로 살펴볼 때 사용하는 **개별 노드 실행법**입니다. 통합 창은 SQLite bag을 직접 읽어 재생·정지·탐색하며, 별도 `ros2 bag play` 프로세스나 토픽 발행을 만들지 않습니다. 통합 실행 파일 `classroom` 추가로 패키지는 1개, 실행 파일은 7개입니다.

ROS 2 패키지 **하나**로, 같은 rosbag 카메라 프레임의 각 처리 단계 전후를 비교합니다. 첫 PPT의 ‘인지 → 판단 → 제어’ 설명 순서를 따릅니다.

| 파일 / 실행 이름 | BEFORE | AFTER | 역할 |
| --- | --- | --- | --- |
| `segmentation_view.py` / `segmentation_view` | 원본 카메라 | PIDNet 클래스별 오버레이 | 판단: 필요한 픽셀 추출 |
| `scan_line_view.py` / `scan_line_view` | PIDNet 차선 마스크 | scan line, 기준 x, 측정 x, 픽셀 오차 | 판단: 목표 조향각의 근거 계산 |
| `pipeline_after_view.py` / `pipeline_after_view` | — | segmentation·scan line·PID 결과를 한 화면에 표시 | 주 교육: AFTER 전용 통합 화면 |
| `reference_color_filter_view.py` / `reference_color_filter_view` | 원본 카메라 | RGB·HSV·YCrCb 필터 결과, 실시간 슬라이더 | 참고자료 전용 |
| `reference_sliding_window_view.py` / `reference_sliding_window_view` | PIDNet 차선 마스크 | 검색창·검출점·추적 곡선 | 참고자료 전용 |
| `pid_view.py` / `pid_view` | 목표 조향각과 기록된 현재 조향각 | P/I/D 항과 조향 모터 PWM | 제어: 원본 drive_control PID 계산 |

이 bag에는 `/arduino/steering_raw`가 실제로 **539개** 있으며 A1 값 범위는 **382~558**입니다. 기본 PID 화면은 이 기록된 피드백으로 PWM을 **재계산**합니다. 새 목표 조향각에 차량이 다시 반응하는 폐루프 실험은 아니며, 원래 기록된 모터 PWM과 같다는 뜻도 아닙니다. 별도 `--feedback demo` 모드는 `DEMO simulated`라고 표시합니다. PIDNet의 ‘PID’와 모터 PID 제어기는 별개입니다.

## 1. ROS 2와 가상환경

운영체제별 ROS 설치는 [INSTALL_ROS2.md](INSTALL_ROS2.md)의 토글을 펼쳐 보세요. Python 버전은 ROS를 빌드/설치한 Python과 맞아야 합니다. 저장소를 내려받는 방법은 [README](README.md#2-가상환경-세팅--ros2-설치)를 참고하세요. 아래 명령은 모두 **프로젝트 폴더(`README.md`가 있는 폴더)의 터미널**에서 실행합니다. 다운로드한 폴더 이름이나 위치는 자유입니다. `~/ros2_jazzy`는 macOS 설치 안내에서 정한 각자의 ROS 빌드 폴더이며, `/opt/ros/jazzy`는 Ubuntu 패키지의 설치 경로입니다.

가상환경 생성과 의존성 설치는 최초 준비 시에만 필요합니다. 이미 설치와 빌드를 마쳤다면 아래 「매 새 터미널」의 환경 활성화 명령만 실행하면 됩니다. `venv` 생성 중 `Ctrl+C`를 누르면 `ensurepip` 단계가 중단되어 설정이 불완전할 수 있습니다.

### macOS (기존 Jazzy 소스 빌드 사용)

```zsh
source ~/ros2_jazzy/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ ! -x .venv/bin/python ]; then
  ~/ros2_jazzy/.venv/bin/python -m venv --system-site-packages .venv
fi
source .venv/bin/activate
python -m pip install -r requirements.txt
touch .venv/COLCON_IGNORE
```

### Ubuntu 24.04 / Windows WSL2 Ubuntu 24.04

저장소를 저장한 위치로 이동한 후 실행합니다. 시스템 ROS와 연결하려고 `--system-site-packages`를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
touch .venv/COLCON_IGNORE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

CPU에서도 실제 PIDNet이 실행됩니다. `--device auto`는 CUDA → MPS → CPU 순서로 선택합니다. CUDA PyTorch 설치는 하드웨어에 맞는 [PyTorch 공식 설치 안내](https://pytorch.org/get-started/locally/)를 따르세요. GPU가 필수는 아니며 속도는 기기마다 다릅니다. 화면은 `PySide6-Essentials`의 Qt Widgets로 표시하고, OpenCV는 영상 처리에 사용합니다. `requirements.txt`에 두 의존성이 포함되어 있습니다. 기존 가상환경에서도 `python -m pip install -r requirements.txt`를 다시 실행하세요.

## 2. 패키지 빌드

가상환경 활성화와 ROS setup을 적용한 터미널에서:

```bash
colcon build --symlink-install --packages-select session_1
```

macOS:

```zsh
source install/setup.zsh
ros2 pkg executables session_1
```

Ubuntu/WSL:

```bash
source install/setup.bash
ros2 pkg executables session_1
```

매 **새 터미널**마다 저장소 루트에서 환경을 불러오세요. `.venv`만 활성화해서는 ROS 패키지가 등록되지 않습니다.

macOS:

```zsh
source ~/ros2_jazzy/install/setup.zsh
source .venv/bin/activate
source install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Ubuntu/WSL:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`--symlink-install`에서 기존 Python 함수 수정은 재시작하면 반영됩니다. 실행 파일 등록, 파일 추가, 의존성/패키지 메타데이터 변경 뒤에는 다시 빌드하고 setup을 불러오세요.

## 3. 지정 rosbag 재생

대상: 저장소 루트의 `rosbag2_2026_08_05-11_29_45/`. `metadata.yaml`과 `.db3`를 함께 유지합니다. 약 97초의 SQLite bag입니다. 상단 카메라 토픽을 사용합니다.

터미널 A에서 아래 시각화 중 하나를 먼저 실행하고, 터미널 B에서 카메라와 조향 피드백 토픽만 재생합니다. 교육 범위에 불필요한 기록된 모터 명령은 재생하지 않습니다.

```bash
ros2 bag info rosbag2_2026_08_05-11_29_45
ros2 bag play rosbag2_2026_08_05-11_29_45 --clock --loop --topics /camera/high/image_raw /arduino/steering_raw
```

처리가 느리면 `--rate 0.5`를 추가합니다. 각 화면은 최신 프레임을 처리하므로 기기에 따라 중간 프레임을 건너뜁니다. PID 시간 간격은 영상 header 시각을 사용하며 bag 반복으로 시간이 되돌아오면 PID와 그래프를 초기화합니다.

### ① Segmentation before / after

```bash
ros2 run session_1 segmentation_view
```

왼쪽은 원본, 오른쪽은 모델 분류를 원본에 겹친 결과입니다. 6개 클래스: 배경, 도로, 실선, 점선, 초록 매트, 밝은 회색. 학습 가중치와 `dataset_info.json`을 포함했으므로 별도 모델 저장소 복제가 필요 없습니다.

### ② Scan line before / after

```bash
ros2 run session_1 scan_line_view --scan-y 0.75 --target-x 0.79
```

왼쪽은 모델이 추출한 선택 차선 마스크입니다. 오른쪽의 가로선은 측정 위치, 주황 세로선은 목표 위치, 노란 점은 측정 위치입니다. `error_px = target_x - measured_x`. 미검출은 `LANE LOST`이며 오차 0과 구분합니다.

기준값은 카메라/해상도/트랙마다 다시 설정해야 합니다. `--target-x 0.79`는 화면 폭의 79%이고, 우측 실선이 정상 주행 시 놓여야 하는 위치입니다. 값은 **BEV를 생략한 원본 카메라 좌표** 기준입니다. 원본의 BEV+근접 띠+초록 매트 검증을 수업용 단일 가로 띠로 단순화했으므로 원본 주행 성능과 동일하다고 볼 수 없습니다.

중앙 점선으로 설명하려면:

```bash
ros2 run session_1 scan_line_view --lane center --target-x 0.35
```

### ③ PID before / after

```bash
ros2 run session_1 pid_view --feedback ros --kp 6.5 --ki 0.0 --kd 0.8 --ros-args -p use_sim_time:=true
```

ROS 입력의 기본값은 기록된 A1 피드백입니다. 화면에서 `RECORDED A1`을 확인하세요. `--clock` 재생과 `use_sim_time:=true`를 함께 사용합니다.

1. 판단: PIDNet과 scan line을 그대로 수행합니다.
2. 판단: 픽셀 오차를 `target_deg = clip(-error_px / 130 × 45, -45, 45)`로 변환합니다(640px 기준, 다른 폭은 비례 보정).
3. 제어: `angle_error = target_deg - actual_deg`에 원본 PID를 적용합니다. `actual_deg`는 기록된 `/arduino/steering_raw`에서 환산합니다.
4. 제어 결과: `P=Kp×각도오차`, `I=Ki×누적오차`, `D=-Kd×측정각속도`. D 각속도에는 alpha=0.25 필터가 있습니다. 출력 PWM은 ±150, 움직임 최소 PWM은 40, 각도 오차 ±1° 이내는 PWM 0입니다. 적분 제한/포화 방지도 원본에서 가져왔습니다.

왼쪽은 PID의 입력, 오른쪽은 출력입니다. 아래 그래프는 목표각(deg)과 PWM의 최근 180개 처리 프레임이며 단위가 다릅니다. D항은 잡음을 키울 수도 있습니다. gain을 바꿔도 기록된 실제 조향각은 바뀌지 않습니다. 재계산 결과만 바뀌며, 차량 안정성 검증에는 실제 폐루프 주행이 필요합니다.

피드백이 없는 다른 bag 또는 모의 축 응답을 설명하려면:

```bash
ros2 run session_1 pid_view --feedback demo
```

모의 축은 `각도 += 이전 PWM × 0.3 × dt`의 단순 예시이며 실제 차량 모델이 아닙니다. 실제 피드백은 `std_msgs/msg/Int16`, 최초 A1을 중앙으로 캡처, 중앙±80이 조향 ±45°인 원본 규약입니다. 피드백이 0.5초 넘게 안 오면 계산 출력을 0으로 표시합니다. 이 패키지는 어떤 모드에서도 실제 모터 명령을 발행하지 않습니다.

## 4. AFTER만 모은 통합 화면

```bash
ros2 run session_1 pipeline_after_view --ros-args -p use_sim_time:=true
```

왼쪽부터 **segmentation 결과 → scan line 측정 결과 → PID PWM 결과**를 나란히 보여줍니다. BEFORE 칸은 없고, segmentation은 프레임당 한 번만 실행합니다. 개별 화면과 같은 파라미터(`--scan-y`, `--target-x`, `--kp`, `--ki`, `--kd`)를 사용하며, PID 피드백 출처도 표시합니다. 화면이 작으면 창을 확대하세요.

## 5. 참고자료 전용: 색 필터와 sliding window

이 두 실행 파일은 이름에 `reference_`를 붙였습니다. 주 교육용 PIDNet → scan line → PID 파이프라인에는 연결하지 않는 별도 비교 자료입니다. 앞서 안내한 동일 bag을 재생하며 실행합니다.

### RGB / HSV / YCrCb 실시간 슬라이더

```bash
ros2 run session_1 reference_color_filter_view
```

원본·RGB·HSV·YCrCb 결과를 2×2로 표시합니다. 같은 창 오른쪽의 RGB / HSV / YCrCb 탭에서 각 채널의 `min` / `max`를 움직입니다(총 18개 슬라이더).

- RGB: R/G/B 각각 0~255. OpenCV 입력 BGR을 RGB로 변환한 뒤 적용합니다.
- HSV: H는 **0~179**, S/V는 0~255. H는 OpenCV 범위이며 0~360도가 아닙니다.
- YCrCb: Y/Cr/Cb 각각 0~255. Y는 밝기, Cr/Cb는 색차입니다.
- 범위에 포함된 픽셀만 원래 색으로 남기고 나머지는 검정으로 표시합니다. 세 색공간의 결과는 각각 독립입니다.
- 초기값은 밝은 흰색 차선을 골라보는 예시입니다. 하한이 상한보다 커지면 `EMPTY RANGE`와 빈 결과를 표시합니다.
- `Space`로 화면을 멈추거나 bag 재생을 멈춘 뒤에도 슬라이더 변경이 마지막 프레임에 즉시 반영됩니다. 정지 이미지 입력에서도 가능합니다.
- PIDNet 추론을 수행하지 않습니다. `--headless`는 기본 범위로 결과만 계산하며 Qt 창은 만들지 않습니다.

### Sliding window 차선 추적

```bash
ros2 run session_1 reference_sliding_window_view --lane right --windows 9 --margin 50 --min-pixels 20
```

입력은 PIDNet 차선 마스크입니다. 아래쪽 1/3의 픽셀 histogram으로 시작 x를 찾고, 아래에서 위로 검색창을 쌓습니다. 창 안의 차선 픽셀 평균 x로 다음 창을 이동합니다.

- 초록 사각형: 충분한 픽셀이 검출된 창. 주황 사각형: 빈 창/픽셀 부족.
- 하늘색 점: 창 내부 픽셀의 평균 위치. 자홍색 곡선: 3개 이상 유효 창의 중심을 2차식으로 연결한 참고 곡선.
- 빈 창에서는 검색 위치만 유지하고 검출점을 만들지 않습니다. 하단 시작점이 없으면 `LANE LOST`, 유효 창이 부족하면 곡선을 표시하지 않습니다.
- `--margin`은 좌우 검색 폭(px), `--min-pixels`는 창을 이동시키는 데 필요한 최소 픽셀 수입니다. 곡선은 실제 검출 y 범위에서만 표시합니다.
- 중앙 점선 설명: `--lane center`. 점선 사이 빈 구간이나 급커브에서 추적이 끊어질 수 있으며, 실제 차량 제어에는 사용하지 않습니다.

새 실행 파일 등록을 반영하려면 다시 빌드하고 현재 셸의 `install/setup.zsh` 또는 `install/setup.bash`를 불러오세요. 새 화면들도 `--output`, `--headless`, `--source image/video`를 지원합니다. 교육용 출력 토픽 접두사는 각각 `/session_1/reference_color_filter`, `/session_1/reference_sliding_window`, `/session_1/pipeline_after`입니다.

## 조작·출력·문제 해결

- `q` / `Esc` / 창 닫기: 종료. `Space`: 화면 처리 일시정지, 다시 누르면 최신 프레임부터 계속. bag 자체는 계속 재생됩니다. bag을 멈추려면 재생 터미널의 Space를 누르세요.
- `r`: PID 적분 및 그래프 초기화.
- `--device cpu`: MPS/CUDA 문제 시 CPU로 실행.
- `--topic /camera/low/image_raw`: 다른 카메라 선택. bag 재생의 `--topics`도 함께 바꿉니다.
- `--headless --max-frames 30 --output output/result.png`: 창 없이 30프레임 처리 후 마지막 화면 저장.
- `--image-rate 5`: 큰 시각화 이미지 토픽은 기본 최대 5Hz로 발행합니다. GUI와 수치 계산은 새 입력마다 계속 처리하며, 빠른 bag 재생의 통신 부하를 줄입니다.
- ROS 출력: `/session_1/{segmentation,scan_line,pid}/image` (`sensor_msgs/Image`), 같은 접두사의 `/metrics` (`std_msgs/String`, JSON).
- 수신 대기만 뜨면 `ros2 topic list`, `ros2 topic info /camera/high/image_raw`로 확인합니다. 구독은 센서에 맞춘 BEST_EFFORT입니다.
- `No module named rclpy`: ROS setup과 Python 버전 확인. `ros2 run`에서 torch를 못 찾으면 프로젝트 `.venv`를 활성화한 상태로 다시 빌드하세요.
- `rmw_fastrtps_cppexport` 오류: 셸 환경변수가 잘못 합쳐진 경우입니다. `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 다시 실행하세요.

ROS 토픽 재생 없이 이미지/동영상을 사용할 수도 있습니다(같은 코드 경로). 테스트할 이미지를 프로젝트 폴더에 `frame.png`, 동영상을 `video.mp4`로 넣고 해당 명령을 실행하세요. 두 파일은 저장소에 포함된 예제 파일이 아니라 직접 준비하는 입력입니다:

```bash
PYTHONPATH=src/session_1 python -m session_1.segmentation_view --source image --input ./frame.png
PYTHONPATH=src/session_1 python -m session_1.pid_view --source video --input ./video.mp4 --feedback demo
```

## 코드 읽는 순서

`runtime.py`: 인지(카메라 디코딩·최신 프레임 수신) → 각 수업 파일의 판단/제어 함수 → 교육 화면·토픽 출력. 실제 bag의 YUY2와 BGR/RGB/mono 및 압축 영상, 행 패딩을 처리합니다.

`segmentation_view.py`: ImageNet 정규화와 PIDNet 추론. `scan_line_view.py`: 실선/점선 선택과 연속 픽셀 중심. `pid_view.py`: 원본 조향 PID와 구분 표시된 모의 피드백. 한국어 주석의 인지/판단/제어 구획을 따라가세요.

첨부 ROS1 스켈레톤에서는 ‘센서 수신 → 계산 → 출력’ 분리를 참고했습니다. ROS1 명령과 90 중심 servo 값은 이 ROS2/PWM 코드에 섞지 않았습니다. 출처·변경점·모델 해시는 [THIRD_PARTY.md](docs/THIRD_PARTY.md), 검증 기록은 [VALIDATION.md](docs/VALIDATION.md)에 있습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/session_1:${PYTHONPATH}" python -m pytest src/session_1/test -q
```


## 검증 재실행

ROS와 프로젝트 환경을 활성화한 터미널에서 실행합니다. 상세 결과와 미검증 항목은 [VALIDATION.md](docs/VALIDATION.md)에 있습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/session_1/test -q
ROS_DOMAIN_ID=98 ROS_LOG_DIR="$PWD/output/roslogs" python scripts/verify_rosbag.py
python scripts/verify_full_bag.py
```

첫 명령은 ROS 전역 pytest 플러그인을 자동 로드하지 않습니다(로컬 ROS의 `launch_pytest`가 요구하는 별도 패키지와 수업 단위 테스트를 분리). 두 번째는 여섯 노드의 DDS 출력·bag 반복·종료를 검사하고, 세 번째는 상단 카메라 전체 2,075프레임을 실제 PIDNet으로 누락 없이 처리합니다. 검출 횟수는 정답 라벨과 비교한 정확도 지표가 아닙니다.
