# ROS 2 Jazzy 설치

OS별 항목을 펼치세요.

**경로 안내:** `~`는 현재 사용자의 홈 폴더입니다. macOS 명령은 각자의 `~/ros2_jazzy`에 ROS를 준비하고, Ubuntu 명령은 표준 경로 `/opt/ros/jazzy`에 설치합니다. 프로젝트는 어디에 내려받아도 됩니다. 프로젝트 명령은 `README.md`와 `start.sh`가 있는 폴더에서 실행하세요.

이 수업은 ROS1 Noetic이 아닌 **ROS 2 Jazzy**를 사용합니다. 설치가 끝나면 [README.md](README.md)의 프로젝트 가상환경·빌드·bag 재생으로 이어집니다.

<details>
<summary><strong>Windows · WSL2 + Ubuntu 24.04</strong></summary>

Windows 11의 WSL2/WSLg에서 Linux용 ROS와 Qt 시각화 창을 실행합니다. 관리자 PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

요청되면 재부팅하고 Ubuntu를 열어 사용자 계정을 만듭니다. 확인:

```powershell
wsl -l -v
```

Ubuntu-24.04의 VERSION이 2인지 확인합니다. 이후 **Ubuntu 터미널**에서 아래 Linux 토글의 설치 과정을 수행하세요. 프로젝트와 bag은 가능하면 Linux 홈 아래에 두세요(`/mnt/c`보다 파일 I/O에 유리).

WSLg가 활성화되어 있으면 Qt 시각화 창을 Windows 바탕화면에서 볼 수 있습니다. 창이 열리지 않으면 WSL을 업데이트하고 다시 시작한 후 Ubuntu에서 `echo $DISPLAY`를 확인하세요. 초기 수업에는 USB 카메라 연결 없이 제공된 bag만 사용합니다.

공식 자료: [Microsoft WSL 설치](https://learn.microsoft.com/en-us/windows/wsl/install), [WSL GUI 앱](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

</details>

<details>
<summary><strong>Linux · Ubuntu 24.04 apt 설치</strong></summary>

아래는 Ubuntu **24.04 Noble**, amd64/arm64용입니다. 다른 Linux 배포판에 그대로 적용하지 마세요.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo apt update
```

공식 `ros2-apt-source` 패키지로 ROS 저장소/키를 등록합니다.

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt upgrade
sudo apt install -y ros-jazzy-desktop ros-dev-tools python3-venv ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`ros-dev-tools` 의존성 충돌 시 `/etc/apt/sources.list.d/ubuntu.sources`의 Suites에 `noble noble-updates noble-backports`가 포함되어 있는지 확인하세요.

터미널 A:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

터미널 B:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
```

`Hello World` 수신을 확인하고 Ctrl+C로 종료합니다. 새 터미널마다 setup을 불러온 뒤 프로젝트 `.venv`와 `install/setup.bash`를 적용합니다.

공식 자료: [Jazzy Ubuntu 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [동일 문서의 공식 원문](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst), [apt 저장소 설정 원문](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst).

</details>

<details>
<summary><strong>macOS · Apple Silicon M4 / Jazzy 직접 빌드 환경</strong></summary>

### 제공된 성공 기록

사용자가 제공한 M4 환경 기록: ROS 패키지 201개, Python 3.10.20 / CMake 3.31.10, `ros2`, `rclpy`, `PyKDL` 정상. Fast DDS와 Cyclone DDS가 설치되었고 Fast DDS의 C++ talker → Python listener `Hello World: 1` 수신에 성공했습니다. 이는 **해당 환경의 성공 기록**이며 새 Mac에서 모든 패키지가 무수정으로 빌드된다는 보장은 아닙니다.

macOS 26/Xcode 26 때문에 RViz/OGRE 및 Python Qt 기반 rqt는 제외했고, PyKDL vendor CMake와 iceoryx binding CMake에 호환성 보정이 적용된 트리입니다. 이 수업은 RViz/rqt 대신 프로젝트 가상환경에 설치한 `PySide6-Essentials`의 Qt 창을 사용합니다. ROS 소스 빌드에서 제외한 rqt와는 별개의 의존성입니다.

### ROS 2 빌드가 완료된 macOS에서 사용

매 새 터미널에서:

```zsh
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 pkg list
```

이후 다운로드한 프로젝트 폴더에서 [README](README.md)의 가상환경·빌드를 진행합니다. 준비가 끝나면 `bash start.sh`가 실행에 필요한 환경을 자동으로 불러옵니다. ROS 빌드 가상환경과 프로젝트 가상환경은 구분하되 Python 3.10을 맞춥니다.

### 기존 보정 트리에서 성공한 재빌드 순서

아래는 사용자가 제공한 성공 명령입니다. 새 설치용 전체 스크립트가 아니라 **기존 `~/ros2_jazzy` 트리** 재빌드 절차입니다.

```zsh
cd ~/ros2_jazzy
source .venv/bin/activate
export OPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
export CMAKE_PREFIX_PATH="$(brew --prefix qt@5)"
export PATH="$(brew --prefix qt@5)/bin:$PATH"

colcon build --symlink-install \
  --packages-skip rviz_ogre_vendor cyclonedds python_qt_binding \
  --packages-skip-by-dep rviz_ogre_vendor cyclonedds python_qt_binding

colcon build --symlink-install \
  --packages-select iceoryx_binding_c cyclonedds

colcon build --symlink-install \
  --packages-up-to ros2cli demo_nodes_cpp demo_nodes_py

colcon build --symlink-install \
  --packages-up-to ros2run ros2pkg
```

설치된 CLI 확장도 확인합니다:

```zsh
source install/setup.zsh
ros2 topic --help
ros2 launch --help
ros2 node --help
ros2 param --help
ros2 service --help
ros2 action --help
ros2 interface --help
ros2 bag --help
```

아직 없는 확장이 있다면 해당 소스가 `src/`에 있는지 확인한 뒤 필요한 패키지까지 빌드합니다. 다음은 이미 설치된 항목의 재빌드에도 사용할 수 있습니다.

```zsh
colcon build --symlink-install --packages-up-to ros2topic ros2launch ros2node ros2param ros2service ros2action ros2interface
colcon build --symlink-install --packages-up-to ros2bag rosbag2_transport rosbag2_storage_sqlite3
source install/setup.zsh
```

검증 터미널 A:

```zsh
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run demo_nodes_cpp talker
```

터미널 B:

```zsh
source ~/ros2_jazzy/.venv/bin/activate
source ~/ros2_jazzy/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run demo_nodes_py listener
```

### 새 Mac에서 처음 설치할 때

Ubuntu 가상머신 없이 macOS에서 직접 빌드할 수 있습니다. Git·Homebrew·Python 설치부터 소스 다운로드·빌드·통신 확인까지의 명령은 [README의 2번 macOS 토글](README.md#2-가상환경-세팅--ros2-설치)에 모았습니다.

새 설치 안내는 수업에 필요한 패키지와 Fast DDS를 선택해 빌드합니다. 위의 기존 전체 빌드 기록과 달리 RViz/rqt·PyKDL·Cyclone DDS를 빌드하지 않습니다. 따라서 PyKDL/iceoryx의 로컬 보정에 의존하지 않습니다. 패키지 선택은 현재 소스에서 확인했으며, 빈 Mac에서 전체 설치를 다시 재현한 것은 아닙니다.

### bag/시각화 의존성 확인

사용자 제공 기록상 SQLite bag 재생 런타임과 GUI OpenCV 5.0.0 설치가 완료되었습니다. 현재 수업 화면은 Qt를 사용하므로 프로젝트 README의 가상환경 설정과 `pip install -r requirements.txt`도 적용하세요. 현재 환경은 다음으로 확인합니다.

```zsh
ros2 bag --help
python -c "import cv2, PySide6; print(cv2.__version__, PySide6.__version__)"
ros2 topic list
ros2 node list
```

`ros2 topic Clist`가 아니라 `ros2 topic list`입니다. 패키지 실행 시 `source install/setup.zsh`까지 적용해야 합니다. rosbag은 디렉터리의 `metadata.yaml`과 `.db3`를 함께 유지하세요.

</details>
