# ORDA · 처음 만나는 자율주행

ROS 2와 자율주행이 처음이어도 괜찮습니다. **명령어 하나로 창을 열고, 버튼을 누르며 배워보세요.**

## 1. 프로젝트 개요

차량이 녹화한 영상을 보며 **보고 → 판단하고 → 움직임을 계산하는 과정**을 살펴봅니다.

> <details>
> <summary>무엇을 배우나요?</summary>
>
> - **인지**: 카메라가 본 장면을 받아요.
> - **판단**: 사진에서 도로·차선을 찾고, 차선 위치를 기준과 비교해요.
> - **제어**: 오차가 0에 수렴하도록 조향값과 속도값을 조절해요. 이 수업 화면에서는 그중 **조향 제어**를 살펴봅니다.
>
> **ROS 2**는 센서와 프로그램이 정보를 주고받게 하는 도구예요. **rosbag**은 그 정보를 저장한 녹화 파일이에요. 이 수업에서는 실제 차량 없이 녹화 파일로 연습합니다.
>
> </details>

## 2. 가상환경 세팅 & ROS2 설치

**Git·Python·라이브러리가 하나도 없는 상태에서 시작하는 순서입니다.** 설치가 끝나면 매번 다시 설치하지 말고 3번만 실행하세요.

| 내 컴퓨터 | 먼저 할 일 | 그다음 |
|---|---|---|
| Windows 11 | 아래 Windows 토글에서 WSL2 설치 | 「공통: Ubuntu에서 설치」 |
| macOS · 새로 시작 | 아래 macOS 토글에서 ROS 2 직접 빌드 | 「macOS: 프로젝트 설치」 |
| Linux · Ubuntu 24.04 | Ubuntu 터미널 열기 | 「공통: Ubuntu에서 설치」 |
| macOS · ROS 2 빌드 완료 | 아래 「macOS: 프로젝트 설치」 | 3번 실행 |

**블록을 위에서 아래로 하나씩 복사하세요.** 명령이 끝나고 입력 커서가 돌아오면 다음 블록으로 넘어갑니다. `sudo` 비밀번호는 입력해도 화면에 보이지 않습니다. 다운로드·설치에는 인터넷과 여유 디스크 공간이 필요합니다. VS Code는 필수가 아닙니다.

> <details>
> <summary>Windows — WSL2 설치부터 시작</summary>
>
> **① Windows의 PowerShell을 관리자 권한으로 열고 실행**
>
> ```powershell
> wsl --install -d Ubuntu-24.04
> ```
>
> 요청되면 Windows를 재부팅하세요. 시작 메뉴에서 **Ubuntu 24.04**를 열어 Linux 사용자 이름과 비밀번호를 만듭니다. Windows 계정과 달라도 됩니다.
>
> **② 다시 PowerShell에서 확인**
>
> ```powershell
> wsl --update
> wsl -l -v
> ```
>
> `Ubuntu-24.04`의 VERSION이 `2`이면 준비 완료입니다. `1`이면 아래를 실행하세요.
>
> ```powershell
> wsl --set-version Ubuntu-24.04 2
> ```
>
> **③ 이제 Ubuntu 터미널에서 아래 「공통: Ubuntu에서 설치」를 진행하세요.** 그 안에서 Git과 Python도 설치합니다. Windows용 Git/Python을 별도로 설치할 필요는 없습니다. GUI는 WSLg를 통해 Windows 바탕화면에 표시됩니다.
>
> 설치가 0%에서 멈추면 `wsl --install --web-download -d Ubuntu-24.04`로 다시 시도하세요. 가상화 관련 오류는 BIOS/UEFI의 가상화 설정을 확인하세요.
>
> 공식 안내: [WSL 설치](https://learn.microsoft.com/en-us/windows/wsl/install), [WSLg GUI](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).
>
> </details>

> <details>
> <summary>macOS — Git·Python 설치부터 ROS 2 직접 빌드까지</summary>
>
> **Ubuntu 가상머신 없이 Mac에 직접 설치합니다.** 아래는 Apple Silicon(M1 이후)과 기본 zsh 터미널 기준입니다. 이미 `~/ros2_jazzy/install/setup.zsh`가 있다면 이 토글을 건너뛰고 「macOS: 프로젝트 설치」로 이동하세요.
>
> **① Mac의 터미널에서 개발 도구 설치**
>
> ```zsh
> xcode-select --install
> ```
>
> 설치 창에서 완료될 때까지 기다리세요. 이미 설치됐다는 메시지가 나오면 다음으로 넘어갑니다.
>
> **② Homebrew·Git·Python·빌드 라이브러리 설치**
>
> ```zsh
> /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
> ```
>
> 설치 안내에 따라 비밀번호와 Enter를 입력한 뒤 실행하세요.
>
> ```zsh
> eval "$(/opt/homebrew/bin/brew shellenv)"
> brew install git python@3.10 ninja pkg-config openssl@3 asio tinyxml2 \
>   eigen console_bridge spdlog yaml-cpp sqlite zstd lz4
> git --version
> ```
>
> 새 터미널에서도 Homebrew를 쓰려면 설치 완료 화면의 **Next steps**에 나온 설정을 적용하세요.
>
> **③ ROS 빌드용 가상환경 준비**
>
> ```zsh
> mkdir -p ~/ros2_jazzy/src
> cd ~/ros2_jazzy
> "$(brew --prefix python@3.10)/bin/python3.10" -m venv .venv
> source .venv/bin/activate
> python -m pip install --upgrade pip
> python -m pip install 'cmake==3.31.10' 'setuptools>=65,<81' \
>   colcon-common-extensions vcstool rosdep 'empy==3.3.4' \
>   catkin_pkg lark PyYAML numpy argcomplete packaging \
>   lxml netifaces psutil pydot pyparsing cryptography \
>   jsonschema importlib-metadata pytest pytest-mock
> touch .venv/COLCON_IGNORE
> python --version
> cmake --version
> ```
>
> Python **3.10**, CMake **3.31.10**인지 확인하세요. ROS를 빌드할 때도, 수업 프로젝트를 실행할 때도 같은 Python 계열을 사용합니다.
>
> **④ ROS 2 Jazzy 소스 내려받기**
>
> ```zsh
> cd ~/ros2_jazzy
> source .venv/bin/activate
> vcs import src --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
> ```
>
> **⑤ 수업에 필요한 ROS 기능 빌드**
>
> 시간이 오래 걸릴 수 있습니다. 마지막에 `Summary:`가 나오고 실패한 패키지가 없는지 확인하세요. 오류가 나면 다음 단계로 넘어가지 말고 첫 실패 패키지의 메시지를 확인합니다.
>
> ```zsh
> cd ~/ros2_jazzy
> source .venv/bin/activate
> export OPENSSL_ROOT_DIR="$(brew --prefix openssl@3)"
> export CMAKE_PREFIX_PATH="$(brew --prefix)"
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> colcon build --symlink-install --parallel-workers 2 \
>   --packages-up-to \
>     ros2run ros2pkg ros2topic ros2launch ros2node ros2param \
>     ros2service ros2action ros2interface ros2bag \
>     rosbag2_transport rosbag2_storage_sqlite3 \
>     demo_nodes_cpp demo_nodes_py sensor_msgs rmw_fastrtps_cpp \
>   --packages-ignore \
>     cyclonedds rmw_cyclonedds_cpp iceoryx_binding_c iceoryx_posh iceoryx_hoofs \
>     rmw_connextdds rmw_connextdds_common rmw_connextddsmicro rosidl_generator_rs \
>   --cmake-args -DBUILD_TESTING=OFF \
>     -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
>     -DPython_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
>     -DRMW_IMPLEMENTATION=rmw_fastrtps_cpp
> ```
>
> 센서 메시지·Python 노드·표준 ROS 명령·SQLite bag 기능을 준비합니다. 통신에는 **Fast DDS**를 사용합니다. 수업에서 쓰지 않는 RViz/rqt·PyKDL·Cyclone DDS는 빌드하지 않으므로, 기존 전체 빌드의 PyKDL/iceoryx 보정은 이 절차에 필요하지 않습니다. 수업 GUI는 프로젝트에서 설치하는 PySide6로 열립니다.
>
> **⑥ 설치 확인**
>
> ```zsh
> source ~/ros2_jazzy/.venv/bin/activate
> source ~/ros2_jazzy/install/setup.zsh
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> python -c "import rclpy; from sensor_msgs.msg import Image; print('ROS Python OK')"
> ros2 pkg list
> ros2 topic --help
> ros2 bag --help
> ros2 run demo_nodes_cpp talker
> ```
>
> talker를 실행한 채 **새 Mac 터미널**에서 다음을 실행하세요.
>
> ```zsh
> source ~/ros2_jazzy/.venv/bin/activate
> source ~/ros2_jazzy/install/setup.zsh
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> ros2 run demo_nodes_py listener
> ```
>
> `Hello World` 수신을 확인하면 두 터미널에서 **Ctrl+C**로 종료하세요. 이어서 아래 「macOS: 프로젝트 설치」를 진행합니다.
>
> **검증 범위:** 기존 M4 직접 빌드 환경은 실행 검증됐습니다. 위 명령의 패키지 선택은 현재 소스에서 확인했지만, 이 설치 절차 전체를 빈 Mac에서 새로 빌드한 것은 아닙니다. macOS/Xcode·Homebrew·Jazzy 소스 버전에 따라 추가 조정이 필요할 수 있습니다. Intel Mac은 이 안내의 검증 대상이 아닙니다.
>
> 참고: [ROS 공식 macOS 소스 빌드 문서](https://docs.ros.org/en/jazzy/Installation/Alternatives/macOS-Development-Setup.html), [Homebrew](https://brew.sh/), [기존 전체 빌드 기록](INSTALL_ROS2.md). 공식 macOS 문서는 오래된 OS 기준이므로, 이 수업에서는 위의 Python/CMake 버전과 제한된 패키지 구성을 사용합니다.
>
> </details>

> <details>
> <summary>공통: Ubuntu에서 설치 — Windows WSL2 / Linux</summary>
>
> **이 토글은 Ubuntu 터미널에서 실행합니다. PowerShell이나 Mac 터미널에 붙여 넣지 마세요.** Linux 사용자는 **Ubuntu 24.04 데스크톱**이 설치된 상태를 기준으로 합니다. 다른 배포판이나 22.04/26.04에는 그대로 적용하지 마세요.
>
> **① 버전 확인**
>
> ```bash
> . /etc/os-release
> echo "$PRETTY_NAME"
> uname -m
> ```
>
> `Ubuntu 24.04`이고 CPU가 `x86_64` 또는 `aarch64`인지 확인하세요.
>
> **② Git·Python·컴파일 도구·GUI 라이브러리 설치**
>
> ```bash
> sudo apt update
> sudo apt install -y curl ca-certificates locales software-properties-common
> sudo add-apt-repository -y universe
> sudo apt update
> sudo apt install -y git build-essential cmake python3 python3-pip python3-venv python3-dev \
>   libgl1 libegl1 libglib2.0-0t64 \
>   libxkbcommon-x11-0 libxcb-cursor0 libxcb-icccm4 libxcb-image0 \
>   libxcb-keysyms1 libxcb-render-util0 libxcb-xinerama0 libxcb-xkb1 \
>   libxcb-randr0 libxcb-shape0 libxcb-xfixes0 libxcb-sync1 \
>   libx11-xcb1 libxrender1 libxi6 libsm6 fonts-noto-cjk
> sudo locale-gen en_US en_US.UTF-8
> sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
> export LANG=en_US.UTF-8
> git --version
> python3 --version
> ```
>
> Python은 Ubuntu 24.04의 **시스템 Python 3.12**를 사용합니다. 별도의 Anaconda/Python 최신 버전을 설치하지 않아도 됩니다.
>
> **③ ROS 2 저장소 등록**
>
> ```bash
> ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
> curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
> sudo dpkg -i /tmp/ros2-apt-source.deb
> ```
>
> **④ ROS 2 Jazzy 설치·확인**
>
> ```bash
> sudo apt update
> sudo apt upgrade -y
> sudo apt install -y ros-jazzy-desktop ros-dev-tools \
>   ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-default-plugins
> source /opt/ros/jazzy/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> ros2 pkg list
> ros2 bag --help
> /usr/bin/python3 -c "import rclpy; print('ROS Python OK')"
> ```
>
> **⑤ 프로젝트 내려받기**
>
> ```bash
> mkdir -p ~/projects
> cd ~/projects
> git clone https://github.com/hello-osy/2026-orda-education.git
> cd 2026-orda-education
> ```
>
> 이미 다운로드했다면 이 블록은 건너뛰고 **`README.md`와 `start.sh`가 있는 폴더의 터미널**에서 이어가세요. `git clone` 전에 `git init`을 실행하지 않습니다.
>
> **⑥ 프로젝트 가상환경과 Python 라이브러리 설치**
>
> 가상환경은 이 프로젝트의 Python 도구를 담는 폴더입니다. `--system-site-packages`는 설치된 ROS Python 도구도 함께 사용하게 합니다.
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> if [ ! -x .venv/bin/python ]; then
>   /usr/bin/python3 -m venv --system-site-packages .venv
> fi
> source .venv/bin/activate
> python -m pip install --upgrade pip
> sed 's/^opencv-python>/opencv-python-headless>/' requirements.txt > .venv/requirements-linux.txt
> python -m pip install -r .venv/requirements-linux.txt
> touch .venv/COLCON_IGNORE
> ```
>
> Linux에서는 Qt 중복을 피하도록 영상 처리용 **OpenCV headless**를 선택합니다. GUI는 별도 라이브러리 **PySide6**로 열리므로 시각화 창은 정상적으로 사용할 수 있습니다. PyTorch·NumPy·PySide6·colcon 등도 이 명령으로 함께 설치합니다. CUDA 설치는 필수가 아닙니다.
>
> **⑦ 패키지 빌드·설치 확인**
>
> 빌드는 ROS가 수업 프로그램을 실행할 수 있도록 등록하는 과정입니다. 아래 명령은 가상환경의 Python으로 빌드합니다.
>
> ```bash
> python "$(command -v colcon)" build --symlink-install --packages-select session_1
> source install/setup.bash
> ros2 pkg executables session_1
> python -c "import rclpy, cv2, torch, PySide6; print('수업 라이브러리 OK')"
> ```
>
> `session_1 classroom`을 포함한 실행 파일들이 나오면 준비 완료입니다. **3번에서 녹화 파일을 준비한 뒤 `bash start.sh`를 실행하세요.** WSL에서는 이후에도 Ubuntu 터미널을 사용합니다.
>
> 근거: [ROS Jazzy 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), [공식 저장소 등록 명령](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst), [Qt Linux 의존성](https://doc.qt.io/qt-6/linux-requirements.html), [OpenCV 패키지 선택](https://pypi.org/project/opencv-python/).
>
> </details>

> <details>
> <summary>macOS: 프로젝트 설치 — 위의 ROS 빌드 완료 후</summary>
>
> **위에서 ROS 빌드를 마쳤거나 이미 설치된 Mac에서 진행합니다.** `~/ros2_jazzy/install/setup.zsh`와 `~/ros2_jazzy/.venv/bin/python`이 있는 경우에 사용하세요. 직접 빌드의 기존 성공 기록과 제약은 [INSTALL_ROS2.md](INSTALL_ROS2.md)에 있습니다.
>
> Git이 없다면 위 macOS 토글의 ①·②를 먼저 진행하세요. 아래 명령은 **Mac 터미널**에서 실행합니다.
>
> ```zsh
> mkdir -p ~/projects
> cd ~/projects
> git clone https://github.com/hello-osy/2026-orda-education.git
> cd 2026-orda-education
> ```
>
> 이미 프로젝트가 있다면 위 블록을 건너뛰고 해당 폴더에서 진행하세요.
>
> ```zsh
> source ~/ros2_jazzy/install/setup.zsh
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> if [ ! -x .venv/bin/python ]; then
>   ~/ros2_jazzy/.venv/bin/python -m venv --system-site-packages .venv
> fi
> source .venv/bin/activate
> python -m pip install --upgrade pip
> python -m pip install -r requirements.txt
> touch .venv/COLCON_IGNORE
> python "$(command -v colcon)" build --symlink-install --packages-select session_1
> source install/setup.zsh
> ros2 pkg executables session_1
> python -c "import rclpy, cv2, torch, PySide6; print('수업 라이브러리 OK')"
> ```
>
> 완료되면 3번으로 넘어가세요. 이 경로에서는 Mac 바탕화면에 GUI가 열립니다.
>
> </details>

> <details>
> <summary>설치 중 막혔을 때</summary>
>
> - **Git 명령을 찾지 못함**: Windows는 Ubuntu 안에서 `sudo apt install -y git`, Mac은 위 Homebrew 설정 후 `brew install git`을 실행하세요.
> - **폴더가 이미 존재함**: 다시 clone하지 말고 기존 프로젝트 폴더에서 가상환경 단계로 이어가세요.
> - **`externally-managed-environment`**: `source .venv/bin/activate` 후 다시 설치하세요. `sudo pip`를 쓰지 않습니다.
> - **`rclpy` 없음**: ROS setup을 먼저 불러오세요. Ubuntu는 `/usr/bin/python3`, Mac 직접 빌드는 ROS를 빌드한 Python으로 만든 가상환경이어야 합니다.
> - **Ubuntu의 apt 의존성 충돌**: `/etc/apt/sources.list.d/ubuntu.sources`에서 `Suites: noble noble-updates noble-backports`가 포함되었는지 확인하세요. [ROS 공식 안내](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)를 참고하세요.
> - **WSL에서 창이 안 열림**: PowerShell에서 `wsl --update`, `wsl --shutdown`을 실행한 뒤 Ubuntu를 다시 여세요. `wsl --shutdown`은 실행 중인 WSL 작업을 모두 종료합니다.
> - **한글이 네모로 보임**: Ubuntu에서 `sudo apt install -y fonts-noto-cjk` 후 창을 다시 여세요.
>
> 설치 안내는 공식 문서를 확인해 정리했습니다. 현재 저장소의 실제 실행 검증은 기존 macOS 직접 빌드 환경 기준이며, 새 Windows/macOS/Ubuntu 전체 설치를 여기서 재현한 것은 아닙니다.
>
> </details>

## 3. 시각화 화면 띄우기

**처음 실행하기 전에 녹화 파일을 한 번 내려받아 주세요.**

> <details>
> <summary>처음 한 번: rosbag 다운로드·압축 풀기</summary>
>
> 1. [수업 rosbag 다운로드 페이지](https://app.notion.com/p/session1-rosbag-file-3e14af7a2973805b83f9e70ffe7c18b0?source=copy_link)를 열고 압축 파일을 다운로드하세요.
> 2. 다운로드한 파일의 **압축을 풀어 주세요.**
> 3. 안에 있는 `rosbag2_2026_08_05-11_29_45` 폴더를 **`start.sh`가 있는 프로젝트 폴더 안으로** 옮기세요.
>
> 아래처럼 `metadata.yaml`과 `.db3` 파일이 녹화 폴더 바로 안에 있으면 됩니다. 같은 이름의 폴더가 두 겹으로 들어가지 않도록 확인하세요.
>
> ```text
> 2026-orda-education/
> ├── README.md
> ├── start.sh
> └── rosbag2_2026_08_05-11_29_45/
>     ├── metadata.yaml
>     └── rosbag2_2026_08_05-11_29_45_0.db3
> ```
>
> **Windows WSL2 사용자:** Ubuntu 터미널에서 프로젝트 폴더로 이동한 뒤 아래 명령을 실행하면 Windows 파일 탐색기로 해당 폴더가 열립니다. 압축을 푼 녹화 폴더를 여기에 복사하세요.
>
> ```bash
> explorer.exe .
> ```
>
> 이미 위 위치에 녹화 파일이 있다면 다시 다운로드하지 않아도 됩니다.
>
> </details>

**프로젝트 폴더의 터미널 하나에서 아래 명령만 실행하세요.** Windows는 Ubuntu 터미널, macOS는 Mac 터미널을 사용합니다. 환경 설정과 녹화 파일 재생은 자동입니다.

```bash
bash start.sh
```

다운로드 위치나 사용자 이름에 맞춰 명령을 고칠 필요가 없습니다. `start.sh`가 있는 폴더에서 실행하면 됩니다.

> <details>
> <summary>창이 열리면 이렇게 해보세요</summary>
>
> 1. **전체 흐름**에서 세 단계의 결과를 한 번에 보세요.
> 2. **1. 이미지 전처리 & 차선 검출 → 2. 주행 오차 계산 → 3. 조향 안정화**를 차례로 눌러보세요.
> 3. **일시정지**를 누르세요. 1·2단계는 왼쪽 처리 전(BEFORE), 오른쪽 처리 후(AFTER)를 비교합니다. 3단계는 오차 그래프와 P/PID 출력을 봅니다.
> 4. **전후 나란히 보기**를 끄면 처리 후 결과만 크게 볼 수 있어요. 3단계에서는 **오차와 출력 함께 보기**로 전환합니다.
> 5. **한 장 앞으로** 또는 아래 재생 막대로 다른 순간을 살펴보세요.
>
> **재생**으로 이어 보고, **처음부터**로 되돌아가세요. 끝까지 재생하면 자동으로 반복됩니다. **수업 종료**를 누르면 녹화 읽기도 함께 끝납니다.
>
> </details>

> <details>
> <summary>직접 값을 바꿔보세요</summary>
>
> - **주행 오차 계산**: 오른쪽 슬라이더로 측정 높이와 원하는 차선 위치를 바꿔보세요.
> - **조향 안정화**: **수렴 모의 실험**에서 P/PID 오차 곡선이 0에 가까워지는 모습을 비교하세요. **녹화된 오차**로 전환하면 실제 기록의 오차 변화를 볼 수 있어요. 모의 실험은 실제 차량의 반응과 다릅니다.
> - **참고: 색 필터**: RGB·HSV·YCrCb를 선택하고 슬라이더로 남길 색을 조절하세요.
> - **참고: 차선 추적**: 작은 검색창이 차선을 따라가는 모습을 보세요.
>
> 값을 바꿔도 **되돌리기** 버튼으로 기본값을 복원할 수 있어요. 실제 차량을 움직이지 않으며, PID 값을 바꿔도 녹화된 차량의 움직임은 바뀌지 않습니다.
>
> </details>

> <details>
> <summary>실행이 안 되거나 더 자세히 알고 싶다면</summary>
>
> - 설치 안내가 나오면 2번의 설치·빌드를 완료하세요.
> - `No executable found`가 나오면 2번의 빌드를 다시 실행하세요.
> - GPU 실행 오류가 나면 CPU로 실행하세요.
>
> ```bash
> bash start.sh --device cpu
> ```
>
> 모델은 프로젝트에 포함되어 있으며, 녹화 파일은 위 다운로드 토글을 따라 별도로 준비합니다. 녹화 폴더를 이동했다면 프로젝트 안에 옮겨 둔 폴더를 `--bag ./my_bag`처럼 지정하세요(`my_bag`은 해당 녹화 폴더 이름).
>
> [상세 사용법·개별 ROS 노드](docs/USAGE_DETAIL.md) · [검증 기록](docs/VALIDATION.md) · [코드·모델 출처](docs/THIRD_PARTY.md)
>
> </details>
