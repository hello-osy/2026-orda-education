# ORDA 2회차 교육

[공통 설치 안내](../README.md) · [Session 1](SESSION_1.md)

카메라와 라이다로 기록을 만들고, 사진으로 물체 찾기를 가르친 뒤, 자율주행 코드를 수정해 보는 수업입니다.

## 실행

README의 ROS 및 공통 개발환경 설치를 마쳐 프로젝트에 `.venv`가 있는 상태에서 시작합니다. **기존 가상환경을 그대로 사용합니다.** 아래 추가 설치는 Session 2 패키지와 NumPy 호환 학습 라이브러리를 `.venv`에 보완합니다. 가상환경 재생성이나 시스템 Python 패키지 변경은 필요하지 않습니다. `README.md`와 `scripts/`가 있는 프로젝트 루트에서 순서대로 실행하세요.

```bash
# 1. 처음 한 번 설치 (이전 설치에서 학습 오류가 났다면 같은 명령을 다시 실행)
.venv/bin/python -m pip install -r src/session_2/requirements.txt
# 2. 학습 라이브러리 검사: 마지막에 'Session 2 학습 라이브러리 OK' 확인
.venv/bin/python src/session_2/scripts/check_install.py
# 3. 실행 (다음부터는 이 명령만 실행)
bash scripts/session_2.sh
```

Ubuntu 24.04의 README 가상환경은 `--system-site-packages`로 시스템 패키지도 볼 수 있습니다. 이전 설치 조건은 NumPy 1용 시스템 Matplotlib·SciPy를 그대로 사용해 NumPy 2에서 `numpy.core.multiarray failed to import`를 일으킬 수 있었습니다. 위 추가 설치는 Matplotlib·ContourPy·SciPy의 NumPy 2 호환 버전을 가상환경에 설치하여 이 충돌을 방지합니다. NumPy를 수동으로 내리거나 `sudo pip`를 실행하지 마세요. 설정 폴더도 실행 전에 자동 생성됩니다. 설치 후 열려 있던 Session 2 창은 종료하고 다시 실행하세요.

Windows 네이티브의 기존 가상환경에서는 PowerShell에서 같은 순서로 실행합니다(WSL은 위 Ubuntu 명령 사용).

```powershell
.\.venv\Scripts\python.exe -m pip install -r src/session_2/requirements.txt
.\.venv\Scripts\python.exe src/session_2/scripts/check_install.py
.\scripts\session_2.cmd
```

ROS 데몬이나 colcon 빌드 없이 실행합니다. 카메라 기본 설정은 USB 연결 부담을 줄인 320×240, 초당 10장입니다. 필요하면 실행할 때 지정합니다.

```bash
bash scripts/session_2.sh --cameras 0 1 --port /dev/cu.usbserial-0001
bash scripts/session_2.sh --width 320 --height 240 --fps 10
bash scripts/session_2.sh --list-ports
```

### 운영체제별 실행과 장치

| 환경 | 실행 | 카메라 방식 | 시리얼 포트 예시 |
|---|---|---|---|
| macOS | `bash scripts/session_2.sh` | AVFoundation | `/dev/cu.usbserial-…`, `/dev/cu.usbmodem…` |
| Ubuntu 24.04 | `bash scripts/session_2.sh` | V4L2 | `/dev/ttyUSB0`, `/dev/ttyACM0` |
| Windows 11 · README의 WSL2/Ubuntu | Ubuntu 터미널에서 `bash scripts/session_2.sh` | Linux에 연결된 V4L2 장치 | Linux 안의 `/dev/ttyUSB0`, `/dev/ttyACM0` |
| Windows 11 · Windows용 Python 가상환경이 있는 경우 | PowerShell에서 `.\scripts\session_2.cmd` | DirectShow | `COM3`, `COM4` |

Windows 네이티브 실행은 `.venv\Scripts\python.exe`, macOS·Ubuntu·WSL은 `.venv/bin/python`을 사용합니다. 서로 다른 OS에서 만든 가상환경 폴더는 공유할 수 없습니다. 실행 스크립트는 공통 Python 진입점을 호출하며 경로에 공백·한글이 있어도 처리합니다.

- **Ubuntu/WSL**: 카메라가 `/dev/video*`로, 시리얼 장치가 `/dev/ttyUSB*` 또는 `/dev/ttyACM*`로 보여야 합니다. 권한 오류는 로그인 계정의 `video`·`dialout` 그룹과 장치 접근 권한을 확인하세요. GUI는 데스크톱 또는 WSLg에서 실행합니다. Qt `xcb` 오류는 [Qt의 Linux 실행 의존성](https://doc.qt.io/qt-6/linux-requirements.html)을 확인하세요. OpenCV가 설정한 전용 Qt 경로는 실행 시 제거해 PySide6와의 충돌을 방지합니다.
- **WSL2**: Windows에서 USB가 보이는 것만으로 WSL에 연결되지는 않습니다. [Microsoft USB 연결 안내](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)에 따라 전달한 뒤 Linux 장치 목록을 확인하세요. USB 카메라는 WSL 커널의 V4L2/UVC 지원도 필요합니다. Windows의 `COM3`를 WSL 입력란에 그대로 넣지 않습니다.
- **Windows 네이티브**: 설정 → 개인정보 및 보안 → 카메라에서 카메라와 데스크톱 앱 액세스를 허용하세요. 라이다·Arduino는 장치 관리자에 표시된 COM 번호를 선택합니다. COM 장치가 없다면 해당 USB 시리얼 칩의 드라이버를 확인하세요.
- 포트 이름은 컴퓨터마다 달라집니다. GUI의 포트 새로고침으로 실제 장치를 선택하며, 제조사 정보가 없는 Arduino는 직접 선택합니다. 자동 선택만으로 모터를 연결하거나 움직이지 않습니다.

모델은 **CUDA → MPS → CPU** 순서로 사용 가능한 장치를 자동 선택합니다. NVIDIA GPU가 있어도 현재 PyTorch가 CUDA를 사용할 수 없으면 CPU를 사용합니다. GPU용 설치 조합은 [PyTorch 공식 안내](https://pytorch.org/get-started/locally/)를 기준으로 확인하세요. 카메라 방식은 [OpenCV의 OS별 API](https://docs.opencv.org/4.12.0/d4/d15/group__videoio__flags__base.html)를 사용합니다.

macOS에서 `not authorized to capture video`가 나오면 시스템 설정 → 개인정보 보호 및 보안 → 카메라에서 실제 실행한 앱(터미널, VS Code, Codex 등)을 허용하세요. 처음 뜨는 권한 창을 허용한 뒤 `연결 시작`을 다시 누르세요. 계속 실패하면 실행 앱을 완전히 종료하고 다시 실행하세요. 다른 앱에서 허용한 권한이 현재 실행 앱에도 적용되는 것은 아닙니다.

카메라 번호에는 내장 카메라도 포함됩니다. `out device of bound (0-0): 1`은 현재 카메라 번호 0만 인식되고 있다는 뜻입니다. 카메라 2의 USB 연결과 다른 앱의 카메라 사용 여부를 확인하세요. `Monospace` 글꼴 대신 시스템에 설치된 고정폭 글꼴을 자동 선택합니다.

Ubuntu에서는 실행 시 `/sys/class/video4linux`의 기본 영상 노드(index 0)를 찾아 카메라 번호를 자동 선택합니다. C920의 메타데이터 노드(index 1)는 제외합니다. USB 재연결에 따라 실제 영상 장치가 `0`, `3`처럼 떨어진 번호일 수 있으므로 항상 `0`, `1`을 지정하지 마세요. 장치가 두 개 미만이면 남은 칸은 수동 설정용 번호이며, 연결된 카메라가 있다는 의미는 아닙니다. 내장 카메라를 포함해 세 대 이상이면 사용할 번호를 직접 지정하세요. 재연결 뒤에는 프로그램을 다시 실행하거나 화면에서 번호를 바꾸세요.

2026-10-06 Ubuntu 장치 확인에서는 C920 두 대가 `/dev/video0`, `/dev/video3`이었으며, 같은 USB 허브에서 320×240·10 FPS로 두 대 동시 수신을 확인했습니다. 이 연결 상태에서 명시적으로 실행하려면 다음을 사용합니다(재연결하면 번호가 바뀔 수 있음).

```bash
bash scripts/session_2.sh --cameras 0 3 --width 320 --height 240 --fps 10
```

화면 프로그램을 종료한 뒤 아래 명령으로 현재 자동 선택된 두 카메라를 검사할 수 있습니다. 두 항목 모두 `errors: []`, `last_age_s < 2`, `forced_shutdown: false`인지 확인합니다.

```bash
.venv/bin/python src/session_2/scripts/check_sensors.py --width 320 --height 240 --fps 10 --seconds 30
```

## 1. ROS2 bag 녹화 & 재생

**bag 파일**은 카메라 영상과 라이다의 거리 정보를 함께 저장한 기록입니다. **라이다**는 주변 물체까지의 거리를 측정하는 센서입니다.

- 위쪽에는 카메라 1, 카메라 2, 라이다 화면이 나란히 있습니다.
- 라이다의 **최대 표시 거리**는 기본 **2m**이며, 0.5~16m 범위에서 조절할 수 있습니다. 실시간·bag 재생 화면에 함께 적용되며, 녹화 데이터의 측정 범위는 바꾸지 않습니다.
- 두 번째 USB 카메라 초기화 중 기존 카메라 영상이 잠시 멈출 수 있어 최대 10초간 연결을 유지하며 회복을 기다립니다. 2초 이상 새 영상이 없으면 화면에 회복 대기를 표시하고, 추론·주행에는 오래된 영상을 사용하지 않습니다. 10초간 회복되지 않으면 USB·허브·다른 앱 사용 상태를 확인하라는 오류를 표시합니다.
- 각 카메라 번호와 라이다 연결 장치를 고른 뒤 `연결 시작` 또는 `모두 연결`을 누릅니다.
- 아래 `● 녹화 시작`을 누르면 현재 수신 중인 센서가 기록됩니다. `■ 녹화 종료 · 저장`으로 마칩니다.
- `최근 녹화 열기` 또는 `bag 파일 열기`로 기록 폴더를 선택합니다. 같은 위쪽 화면에 저장된 영상과 거리가 표시됩니다.
- `재생`, `일시정지`, `다음 장면`, 재생 위치 막대로 기록을 살펴봅니다.
- `실시간 보기`를 누르면 연결된 센서의 현재 영상으로 돌아갑니다. 화면 아래 문구로 실시간인지 기록 재생인지 구분합니다.

녹화는 항상 **실시간 센서**를 저장합니다. 기록 재생 화면을 녹화하는 기능은 아닙니다. 2·3번으로 이동해도 진행 중인 녹화는 계속됩니다. 녹화 버튼은 1번에만 있습니다.

라이다 각도 표시는 대회 저장소와 같은 **앞 ±180° / 오른쪽 +90° / 뒤 0° / 왼쪽 −90°**입니다. 실시간 A1 원시 각도와 ROS LaserScan 각도 사이에는 `ROS 각도 = 180° − 원시 각도` 변환을 적용합니다. 새 녹화와 대회 SLLIDAR bag은 같은 방향으로 표시됩니다. 2026-10-06 각도 수정 전에 Session 2에서 녹화한 bag은 라이다 재생 화면에서 **각도 기준: 이전 Session 2 녹화**를 선택하세요. 기존 파일은 변경하지 않습니다.

기록 안에 영상이 여러 개 있으면 카메라 1·2 기록과 라이다 기록을 선택할 수 있습니다. 일반적인 파일은 자동으로 선택됩니다.

### 기록 형식

표준 ROS 2 SQLite bag (`metadata.yaml` + `.db3`, CDR)으로 저장합니다.

| 입력 | 기록 이름(토픽) | 메시지 형식 |
|---|---|---|
| 카메라 1 | `/camera/high/image_raw` | `sensor_msgs/msg/Image`, BGR8 |
| 카메라 2 | `/camera/low/image_raw` | `sensor_msgs/msg/Image`, BGR8 |
| 라이다 | `/scan` | `sensor_msgs/msg/LaserScan` |

Image와 CompressedImage, LaserScan 재생 및 SQLite 분할 bag을 지원합니다. MCAP과 파일 압축 bag은 지원하지 않습니다. 재생 중 모터 신호를 보내지 않습니다.

기록 시각은 컴퓨터가 데이터를 받은 시각입니다. 센서를 하드웨어로 동기화하지 않으며, 선택한 시각 이전의 가장 최근 기록을 표시합니다. 오래된 영상은 표시로 구분합니다. 고부하 상태에서는 일부 기록이 빠질 수 있고, 저장 개수와 누락 수는 화면 및 `session2_recording.json`에 남습니다. 저장 완료 후 종료하세요.

## 2. YOLOv8 학습 & 추론

세 영역을 한 화면에서 왼쪽부터 사용합니다.

### 사진 캡처하기

1. `bag 파일 열기`로 녹화 기록을 선택합니다. 1번에서 연 기록도 그대로 사용합니다.
2. **카메라 2 기록**이 선택되어 있는지 확인하고 `사진 캡처하기`를 누릅니다. 카메라 2에서만 수집하며, 카메라 2 기록이 없으면 캡처하지 않습니다.
3. 카메라 2의 5, 10, 15번째 장면이 사진으로 저장됩니다. `사진 폴더 열기`로 확인합니다.
4. `YOLOv8 학습용 데이터셋 만들기 - Roboflow로 이동`을 누르면 수업용 공개 데이터셋 페이지가 열립니다. 직접 데이터셋을 만들 때는 Roboflow 작업 공간에서 사진 속 물체에 사각형을 그리고 이름을 붙입니다. 사진과 정답 표시를 합친 것을 **학습 데이터**라고 합니다.
5. Roboflow에서 YOLOv8 형식으로 내려받고 압축을 풉니다.

같은 촬영의 비슷한 사진이 학습용과 시험용에 섞이지 않도록 촬영 구간별로 나누세요. 업로드와 정답 표시는 본인 Roboflow 프로젝트에서 진행합니다.

### YOLOv8 모델 학습

1. `학습 사진 목록 선택 · data.yaml`에서 내려받은 폴더의 `data.yaml`을 선택합니다.
2. `학습 사진 확인`으로 사진과 정답 파일을 검사합니다.
3. `모델 학습 시작`을 누릅니다. 아래 내역에서 진행 상황을 확인합니다.

설정은 **YOLOv8n, 30회 반복, 사진 크기 640, 한 번에 4장, 장치 자동 선택**으로 고정됩니다. 사용 가능한 CUDA, MPS, CPU 순서로 자동 선택합니다. 최초 학습 시 기본 모델 파일을 다운로드합니다. 학습 중지나 실패는 완료로 표시하지 않습니다.

### YOLOv8 모델 추론

**추론**은 학습한 모델로 영상에서 물체를 찾는 과정입니다.

1. 학습이 끝나면 `best.pt`가 오른쪽에 자동 입력됩니다. 기존 모델은 `모델 파일 선택`으로 엽니다.
2. 입력은 **카메라 2로 고정**됩니다. 카메라 2의 장치 번호는 1번 화면에서 설정합니다.
3. `물체 찾기 시작`을 누르면 카메라 2를 연결하고 실시간으로 물체를 찾습니다. 원본만 보려면 `카메라 2 연결`을 누릅니다.
4. 현재 영상에 물체의 사각형과 이름, 물체 수와 처리 시간이 표시됩니다.
5. `물체 찾기 중지`는 추론을 멈추고 원본 영상을 보여줍니다. `카메라 2 중지`는 카메라 2 연결을 종료합니다. 1번 화면과 같은 카메라 연결을 사용합니다.

bag 파일 없이 실시간 추론을 사용할 수 있습니다. 모델 실행 장치는 자동 선택되며 판단 기준은 0.25로 고정됩니다. 처리 중 들어온 영상은 쌓지 않고 최신 프레임부터 처리합니다. 영상이 2초 이상 들어오지 않으면 이전 결과를 지우고 연결 상태를 안내합니다. 학습을 시작하면 추론은 중지됩니다. YOLOv8의 물체 찾기는 자율주행 코드의 차선 찾기 모델과 별개입니다.

1·2번 화면은 최소 창 크기 1100×760에서 페이지 전체 스크롤 없이 배치합니다. 긴 로그는 로그 상자 안에서 스크롤하고, 긴 상태 문구는 마우스를 올리면 전체 내용을 볼 수 있습니다.

## 3. 자율주행 코드

`autonomous_driving.py` 한 파일에 차선 찾기, 차선 위치 계산, 핸들 방향 계산(PID), 모터 통신을 정리했습니다. 여러 파일을 붙여 읽는 창이 아니라 실제 Python 소스 한 개를 편집하는 창입니다. 신경망 구조와 학습된 차선 모델은 프로젝트의 Session 1 자료를 사용합니다.

코드는 기본 `if/else`와 `for` 문을 중심으로 풀어 썼습니다. `# ========` 구분선과 **이미지 전처리 & 차선 검출 → 주행 오차 계산 → 조향 안정화 → 전체 코드 연결** 제목으로 나눕니다. 각 블록의 주석도 화면 설명의 계산 순서와 맞춥니다.

- 줄 번호와 Python 색상 강조, 자동 들여쓰기, 실행 취소·다시 실행을 지원합니다.
- `⌘F` 또는 `Ctrl+F`: 찾기. Enter로 다음 결과, Esc로 검색창 닫기.
- `저장` 또는 `⌘S` / `Ctrl+S`: `workspace/session_2/autonomous_driving.py`에 저장.
- 다음 실행에서도 저장한 파일을 그대로 엽니다. 처음에는 교육용 원본을 표시합니다.
- 수정한 채 종료하면 저장·버리기·취소 중 선택할 수 있습니다.

코드 편집·저장만 제공하며 자동 실행이나 모터 제어 버튼은 없습니다.

3번 화면 안의 네 버튼으로 내용을 전환합니다. 새 창은 열리지 않습니다.

- **이미지 전처리 & 차선 검출**: 사진 크기·색 정리와 PIDNet 계산 부분
- **주행 오차 계산**: 차선 위치와 목표 바퀴 각도 계산 부분
- **조향 안정화**: PID로 조향 모터 힘을 계산하는 부분
- **전체 코드**: 한 파일 전체를 편집하고 저장하는 화면

왼쪽은 역할과 계산 순서를 짧게 설명하고, 오른쪽은 문법 색상과 스크롤을 갖춘 코드 화면입니다. 설명은 수업 기본 코드 기준입니다. 부분 코드는 읽기 전용이며, 전체 코드의 현재 편집 내용을 반영합니다. 찾기는 현재 보이는 코드에서 동작합니다. 저장은 항상 전체 파일을 저장합니다.

전처리는 이미지 크기 변경 → BGR에서 RGB로 변경 → 데이터 구조 변경 → 모델 추론 → 픽셀별 클래스 선택의 5단계로 설명합니다. 주행 오차는 오른쪽 차선과 기준 위치의 차이를 −45~45도 조향값으로 바꾸며, 조향 안정화에서는 속도에 맞춰 P·D값을 튜닝합니다. 전체 코드 설명은 인지(ROS2 센서 입력) → 판단(segmentation·scan line·조향 안정화) → 제어(ROS2 모터 명령) 흐름을 보여 줍니다. 이 흐름은 ROS2 기반 구성 설명이며, 현재 교육용 파일의 모터 통신은 시리얼 방식입니다.

## 4. 스케일카 굴려보기

기존 GUI 안에서 **왼쪽 segmentation / 가운데 scan line / 오른쪽 위 PID / 오른쪽 아래 YOLO**를 함께 봅니다.

1. 카메라를 고르고 `추론 시작`을 누릅니다. 1번과 같은 카메라 연결을 사용합니다. PIDNet은 도로·차선을 색으로 표시하고, 오른쪽 실선과 목표 위치의 차이로 목표 바퀴 각도를 정합니다.
2. YOLO는 2번에서 고른 모델과 **카메라 2 영상**을 사용합니다. 차선 검출용 카메라 선택과는 별개입니다. 아직 없다면 `YOLO 모델 선택 · best.pt`로 고른 뒤 `추론 시작`을 누릅니다. 모델 변경은 추론과 주행을 중지합니다. YOLO는 별도 작업에서 실행되며 조향 명령에는 사용하지 않습니다. Mac의 MPS에서는 GPU 충돌을 막기 위해 PIDNet과 YOLO를 하나씩 실행하고, 두 화면은 계속 갱신합니다.
3. `모터 보드` 포트를 확인하고 `모터 연결`을 누릅니다. 제조사가 Arduino인 포트가 한 개면 자동 선택합니다. 연결만으로 출발하지 않고 정지 명령을 보냅니다.
4. 바퀴를 똑바로 놓고 `바퀴 중앙 맞추기`를 누릅니다. `45도까지 센서 변화량`은 중앙에서 45도까지의 센서 차이이며 기본값은 기존 코드의 80입니다. 차량에 맞게 확인한 값을 연결 전에 설정합니다.
5. `전진 힘`을 정하고 `주행 시작 · 실제 모터 출력`을 누릅니다. 설정 범위는 150~230 PWM이고 기본값은 150입니다. 출발 시에는 0에서부터 실제 출력은 한 번에 5씩 올라갑니다.
6. `정지 · 모터 출력 0` 또는 이 화면에서 Space를 누르면 출력을 멈춥니다. 다시 시작하려면 `주행 시작`을 다시 눌러야 합니다.

주행 시작 상태에서 **새 영상은 정상적으로 들어오지만 차선을 찾지 못하면**, 목표 바퀴 각도를 중앙(0°)으로 두고 전진 PWM을 **150**으로 제한합니다. 정지 버튼과 통신 끊김 정지는 항상 PWM 0입니다. 차선을 다시 찾으면 차선 조향으로 돌아가고 전진 힘은 제어 주기마다 5씩 회복합니다. PWM은 실제 속도가 아니라 모터 출력이므로 150이 실제 저속에 해당하는지는 차량에서 확인해야 합니다. 사용자 설정에 따라 기본값과 미검출 제한을 모두 150으로 통일했습니다. 카메라·추론·바퀴 피드백·GUI 입력이 오래되면 저속 전진 대신 정지하며, 다시 주행 시작을 눌러야 합니다.

PID 그래프는 목표·실제 바퀴 각도와 조향 PWM을 표시합니다. 실제 각도는 Arduino 피드백에서 구하며, 연결 전에는 목표만 표시합니다. 아래 상태에는 마지막으로 시리얼에 송신한 명령을 표시합니다.

통신 규격은 **115200 baud**, 송신 `조향PWM 전진PWM\n`, 수신 `R,센서값,전압,백분율\n`입니다. GUI 응답이 0.3초, 카메라 추론 입력 또는 바퀴 피드백이 0.5초 넘게 끊기면 제어 스레드가 정지 명령을 보냅니다. 추론 중지·카메라 변경·다른 수업 화면 이동·GUI 종료도 출력을 중지합니다. USB가 물리적으로 끊기면 정지 명령 자체를 전달할 수 없으므로 모터 보드 펌웨어의 통신 끊김 정지 기능도 필요합니다.

4번은 프로젝트의 주행 구현을 실행합니다. 3번에서 편집·저장한 파일을 자동으로 실행하지는 않습니다.

## 저장 위치

```text
workspace/session_2/
├── autonomous_driving.py  # 3번에서 저장한 편집 코드
├── bags/                 # 센서 녹화
├── images/               # 캡처 사진과 원본 기록 목록
├── runs/                 # 학습 내역과 weights/best.pt
└── validation/           # 소프트웨어 검증용 합성 자료
```

녹화·캡처·학습은 매번 새 폴더에 저장합니다. 코드 저장은 같은 `autonomous_driving.py` 파일을 갱신합니다.

## 구현 근거와 검증

참고 저장소: [ai-autonomous-driving-competition-2026](https://github.com/hello-osy/ai-autonomous-driving-competition-2026), 확인한 커밋 `e1a3e382e9d70fa09768869ca5de532549a68900`.

- [카메라 설정](https://github.com/hello-osy/ai-autonomous-driving-competition-2026/blob/e1a3e382e9d70fa09768869ca5de532549a68900/sensor_topic/config/camera.yaml): high/low 토픽
- [Arduino 통신](https://github.com/hello-osy/ai-autonomous-driving-competition-2026/blob/e1a3e382e9d70fa09768869ca5de532549a68900/sensor_topic/sensor_topic/arduino_communication_node.py): 송신·R 피드백 규격
- [통합 펌웨어](https://github.com/hello-osy/ai-autonomous-driving-competition-2026/blob/e1a3e382e9d70fa09768869ca5de532549a68900/arduino_code/integrated_ardunio_code/integrated_ardunio_code.ino): 모터 핀·방향·watchdog
- [YOLOv8](https://docs.ultralytics.com/models/yolov8/) · [학습/MPS](https://docs.ultralytics.com/modes/train/) · [ROS bag 형식](https://ternaris.gitlab.io/rosbags/topics/rosbag2.html)

교육용 통합 코드의 PID와 차선 찾기는 현재 프로젝트의 `src/session_1` 구현을 바탕으로 정리했습니다. 최신 참고 저장소의 PID 기본 게인은 달라질 수 있으며, 이 수업은 **Session 1의 P=6.5**를 유지합니다.

검증 상세: [Session 2 검증 기록](../src/session_2/VALIDATION.md).
