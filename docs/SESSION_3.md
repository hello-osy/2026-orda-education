# ORDA 3회차 교육

[공통 설치](../README.md) · [Session 2](SESSION_2.md)

수강생은 **환경 세팅 → 시뮬레이터 실행 → 명령어 실습** 순서로 진행합니다. 명령은 프로젝트 루트(`README.md`가 있는 폴더)에서 실행합니다.

## 1. 환경 세팅

세션 2의 **기존 `.venv`와 ROS2 Jazzy**를 사용합니다. 해당 OS 항목을 펼쳐 처음 한 번 설치합니다.

<details>
<summary>Ubuntu 24.04 / Windows WSL2 Ubuntu 24.04</summary>

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz
.venv/bin/python -m pip install -r src/session_3/requirements.txt
```

</details>

<details>
<summary>macOS</summary>

```bash
brew tap osrf/simulation
brew trust osrf/simulation
brew install gz-harmonic
.venv/bin/python -m pip install -r src/session_3/requirements.txt
bash scripts/session_3_macos_bridge.sh
```

마지막 명령은 ROS와 Gazebo를 연결하는 패키지를 빌드합니다. 이전 버전에서 빌드가 실패했다면 업데이트 후 마지막 명령을 다시 실행하세요.

</details>

<details>
<summary>WSL 추가 설정: 실차 네트워크 연결</summary>

Windows 11 22H2 이상에서 `%UserProfile%\.wslconfig`에 추가합니다. 기존 `[wsl2]`가 있으면 그 안에 항목만 넣습니다.

```ini
[wsl2]
networkingMode=mirrored
```

실행 중인 WSL 작업을 저장한 뒤 PowerShell에서 `wsl --shutdown`하고 Ubuntu를 다시 엽니다. 방화벽이 막으면 관리자 PowerShell에서 수업용 포트를 허용합니다.

```powershell
New-NetFirewallRule -Name ORDA-ROS2 -DisplayName "ORDA ROS2" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 14900-15899 -RemoteAddress LocalSubnet
New-NetFirewallHyperVRule -Name ORDA-ROS2 -DisplayName "ORDA ROS2" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 14900-15899
```

[WSL 네트워크 안내](https://learn.microsoft.com/en-us/windows/wsl/networking)

</details>

## 2. 시뮬레이터 켜는 명령

아래 명령 하나로 빌드·환경 확인 후 시뮬레이터를 엽니다. 이 터미널은 켜 둡니다.

```bash
bash scripts/session_3.sh
```

## 3. 스케일카와 토픽 주고 받는 방법

Wi-Fi **`AndroidHotspot6679`**, 비밀번호 **`12345osy`**로 접속하면 스케일카와 같은 네트워크에 연결됩니다. 실차를 사용할 터미널에서 도메인도 맞춥니다.

```bash
source scripts/session_3_env.sh 31
```

차량은 카메라·라이다 토픽을 발행하고 `/motor_control`을 구독합니다. 여러 사람이 센서를 볼 수 있지만, 모터 명령은 **오상영이 지정한 한 명씩** 보냅니다.

<details>
<summary>오상영 사전 준비: 차량 연결 노트북 1대에서 실행 (수강생은 건너뛰기)</summary>

USB 장치를 연결한 노트북에서 진행합니다. 노트북 한 대에서만 확인할 때는 핫스팟 없이 사용할 수 있습니다. 세션 2를 종료한 뒤 빌드·포트를 확인합니다.

```bash
source scripts/session_3_env.sh 31
python "$(command -v colcon)" build --symlink-install --packages-select session_3
source scripts/session_3_env.sh 31
python -m serial.tools.list_ports
```

먼저 `arm:=false`로 ROS 제어 출력을 끄고 센서를 확인합니다. Arduino의 전원 초기 중앙 맞춤은 별도로 실행됩니다. 아래는 이번 Mac 연결의 장치 번호·포트이며, 재연결하면 바뀔 수 있습니다.

```bash
ros2 launch session_3 vehicle.launch.py \
  camera_high:=1 camera_low:=2 \
  lidar_port:=/dev/cu.usbserial-0001 motor_port:=/dev/cu.usbmodem31201 \
  arm:=false
```

Ubuntu/WSL에서는 현재 카메라 번호와 `/dev/ttyUSB…`, `/dev/ttyACM…` 포트로 바꿉니다. 같은 노트북의 새 터미널에서 센서를 봅니다.

```bash
source scripts/session_3_env.sh 31
ros2 launch session_3 viewers.launch.py
```

조향 실습은 센서 확인 후 `arm:=true`로 다시 실행합니다. Arduino의 초기 움직임이 멈추고 **중앙 자동 등록 완료 로그**가 뜰 때까지 기다립니다. 노드는 10초 대기 후 2초간 안정된 피드백으로 중앙을 자동 등록합니다(최소 약 12초).

중앙을 다시 등록할 때만 바퀴를 중앙에 놓고 `ros2 service call /motor/calibrate std_srvs/srv/Trigger '{}'`를 실행합니다. 차량 연결 터미널은 켜 둡니다.

수업 전에 ±5도로 조향 방향을 확인합니다. 차량에 따라 `raw_direction`(센서 방향), `pwm_direction`(모터 방향)을 `1` 또는 `-1`로 설정합니다. 통신 끊김 시 정지는 Arduino 펌웨어에서도 지원해야 합니다.

</details>

## 4. ROS2 명령어 따라서 쳐보기

새 터미널에서 진행합니다. `30`은 개인 시뮬레이터, `31`은 공용 실차, `32`는 녹화 재생입니다. `30`·`32`는 내 컴퓨터 안에서만 통신합니다.

<details>
<summary>1. 실행 중인 노드 확인하기</summary>

```bash
source scripts/session_3_env.sh 30
ros2 node list
```

</details>

<details>
<summary>2. 발행 중인 토픽과 메시지 타입 확인하기</summary>

```bash
ros2 topic list -t
ros2 topic info /scan
ros2 interface show sensor_msgs/msg/LaserScan
```

</details>

<details>
<summary>3. 라이다 데이터 출력해 보기</summary>

거리 배열이 한 번 출력됩니다. 단위는 m이며, `inf`는 측정 범위 안에 반환값이 없다는 뜻입니다.

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
```

</details>

<details>
<summary>4. 카메라·라이다 시각화 노드 각각 실행하기</summary>

각각 새 터미널에서 실행합니다. 창을 닫거나 `Ctrl+C`로 종료합니다.

```bash
source scripts/session_3_env.sh 30
ros2 run session_3 camera_viewer
```

```bash
source scripts/session_3_env.sh 30
ros2 run session_3 lidar_viewer
```

</details>

<details>
<summary>5. 모터 제어 토픽을 발행해 시뮬레이터 자동차 움직이고 정지하기</summary>

`[조향각(도), 속도 PWM]`입니다. 조향은 양수=왼쪽, 음수=오른쪽이며 ±30도까지입니다. 아래 명령은 직진합니다. **`Ctrl+C`로 발행을 끝내면 약 0.5초 뒤 멈춥니다.**

```bash
ros2 topic pub -r 10 /motor_control std_msgs/msg/Int16MultiArray "{data: [0, 100]}"
```

명시적인 정지 명령:

```bash
ros2 topic pub --once /motor_control std_msgs/msg/Int16MultiArray "{data: [0, 0]}"
```

</details>

<details>
<summary>6. 실제 스케일카의 센서 토픽 받기</summary>

핫스팟에 접속한 뒤 기존 시각화 창을 닫습니다. 새 터미널에서 실차 도메인으로 실행하면 실차 카메라가 보입니다.

```bash
source scripts/session_3_env.sh 31
ros2 topic list -t
ros2 run session_3 camera_viewer
```

라이다는 다른 터미널에서 실행합니다.

```bash
source scripts/session_3_env.sh 31
ros2 run session_3 lidar_viewer
```

</details>

<details>
<summary>7. 실제 스케일카를 위한 모터 토픽 발행하기 (조향만)</summary>

**오상영이 지정한 순서에 따라 한 명씩 실행합니다.** 속도값은 반드시 `0`입니다. 왼쪽 5도로 조향한 뒤 `Ctrl+C`를 누릅니다. `Ctrl+C`는 모터 출력을 끄며, 바퀴를 중앙으로 돌리지는 않습니다.

조향 PWM은 `±150`의 짧은 펄스로 보내고 새 피드백을 기다립니다. 목표 1도 이내 도달·통과 시 멈추며 같은 목표는 재시작하지 않습니다. 목표 처리 시간은 최대 5초입니다.

```bash
source scripts/session_3_env.sh 31
ros2 topic pub -r 10 /motor_control std_msgs/msg/Int16MultiArray "{data: [5, 0]}"
```

`5`를 `-5`로 바꾸면 오른쪽입니다. 중앙으로 돌리려면 아래 명령을 실행하고, 바퀴가 중앙에 오면 `Ctrl+C`를 누릅니다.

```bash
ros2 topic pub -r 10 /motor_control std_msgs/msg/Int16MultiArray "{data: [0, 0]}"
```

</details>

<details>
<summary>8. 센서 토픽을 bag으로 녹화하고 재생하기</summary>

실차 센서만 녹화합니다. 몇 초 뒤 `Ctrl+C`로 저장을 마칩니다. 같은 폴더가 이미 있으면 이름을 바꿉니다.

```bash
source scripts/session_3_env.sh 31
mkdir -p workspace/session_3/bags
ros2 bag record --storage sqlite3 -o workspace/session_3/bags/practice \
  /scan /camera/high/image_raw /camera/low/image_raw
```

재생은 별도 도메인 `32`에서 합니다. **터미널 1:**

```bash
source scripts/session_3_env.sh 32
ros2 bag play workspace/session_3/bags/practice --clock --loop
```

</details>

<details>
<summary>9. launch 파일로 시각화 노드를 한 번에 실행하기</summary>

**터미널 2:** 녹화한 영상과 라이다가 함께 열립니다.

```bash
source scripts/session_3_env.sh 32
ros2 launch session_3 viewers.launch.py use_sim_time:=true
```

실시간 실차는 도메인 `31`, `use_sim_time:=false`로 실행합니다.

</details>

[검증 범위](../src/session_3/VALIDATION.md)
