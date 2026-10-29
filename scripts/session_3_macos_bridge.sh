#!/usr/bin/env bash
# 기존 Jazzy를 바꾸지 않고, Homebrew Harmonic과 연결할 ROS bridge만 추가한다.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_ROOT="${SESSION3_ROS_ROOT:-$HOME/ros2_jazzy}"
BRIDGE_WS="$PROJECT_DIR/workspace/session_3/bridge"
PYTHON="$PROJECT_DIR/.venv/bin/python"

if [[ "$(uname -s)" != Darwin ]]; then
  echo '이 스크립트는 macOS용입니다. Ubuntu/WSL은 sudo apt install ros-jazzy-ros-gz 를 사용하세요.' >&2
  exit 1
fi
if [[ ! -x "$PYTHON" || ! -f "$ROS_ROOT/install/setup.bash" || ! -d "$ROS_ROOT/src" ]]; then
  echo 'README의 기존 .venv와 ~/ros2_jazzy 소스/설치가 필요합니다. 다른 ROS 경로는 SESSION3_ROS_ROOT로 지정하세요.' >&2
  exit 1
fi
for tool in git brew pkg-config; do
  command -v "$tool" >/dev/null || { echo "필요한 명령 없음: $tool" >&2; exit 1; }
done
BREW_PREFIX="$(brew --prefix)"
export PATH="$PROJECT_DIR/.venv/bin:$BREW_PREFIX/bin:$PATH"
export CMAKE_PREFIX_PATH="$BREW_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="$BREW_PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
if ! pkg-config --exists 'gz-msgs10 >= 10.4.0' 'gz-transport13 >= 13.6.0'; then
  echo '먼저 최신 Homebrew Harmonic을 설치하세요: brew install gz-harmonic (기존 설치: brew upgrade gz-harmonic)' >&2
  echo 'gz-msgs10 >= 10.4.0, gz-transport13 >= 13.6.0이 필요합니다. 구버전에서 Gazebo 전체를 자동 재빌드하지 않습니다.' >&2
  exit 1
fi
source "$ROS_ROOT/install/setup.bash"
source "$PROJECT_DIR/.venv/bin/activate"
"$PYTHON" -c 'import rclpy; import ament_index_python' || {
  echo '프로젝트 .venv는 기존 Jazzy와 같은 Python으로 만들어야 합니다. README의 macOS 설치를 확인하세요.' >&2
  exit 1
}

# Jazzy의 CMake/Python 생성기 조합을 유지한다. 가상환경을 새로 만들지 않는다.
"$PYTHON" -m pip install 'cmake==3.31.10' 'empy==3.3.4' catkin_pkg lark PyYAML colcon-common-extensions
COLCON="$(command -v colcon)"
mkdir -p "$BRIDGE_WS/src"
touch "$BRIDGE_WS/COLCON_IGNORE"
export ROS_LOG_DIR="$BRIDGE_WS/log/ros"
mkdir -p "$ROS_LOG_DIR"
# Homebrew의 같은 major 라이브러리를 재사용한다. Gazebo 전체를 다시 빌드하지 않는다.
export GZ_RELAX_VERSION_MATCH=1
export GZ_BUILD_FROM_SOURCE=''
export ROS_VERSION=2 ROS_PYTHON_VERSION=3 ROS_DISTRO=jazzy

# 소스가 이미 있으면 수정하거나 reset하지 않고, 지정한 태그인지 확인한다.
clone_pinned() {
  local folder="$1" url="$2" ref="$3" target="$BRIDGE_WS/src/$1"
  if [[ ! -e "$target" ]]; then
    git -c advice.detachedHead=false clone --depth 1 --branch "$ref" "$url" "$target"
  elif [[ ! -d "$target/.git" ]]; then
    echo "기존 경로가 Git 저장소가 아닙니다: $target" >&2
    exit 1
  fi
  if [[ -n "$(git -C "$target" status --porcelain)" ]] ||
     [[ "$(git -C "$target" rev-parse HEAD)" != "$(git -C "$target" rev-parse "$ref^{commit}" 2>/dev/null)" ]]; then
    echo "기존 소스를 보존하고 중단합니다. $target 의 변경/버전($ref)을 확인하세요." >&2
    exit 1
  fi
}

clone_pinned ros_gz https://github.com/gazebosim/ros_gz.git 1.0.25
# release 태그는 해당 메시지 패키지만 포함한다. GPS 드라이버/RViz는 설치하지 않는다.
clone_pinned actuator_msgs https://github.com/ros2-gbp/actuator_msgs-release.git release/jazzy/actuator_msgs/0.0.1-4
clone_pinned gps_msgs https://github.com/ros2-gbp/gps_umd-release.git release/jazzy/gps_msgs/3.1.1-1
clone_pinned marine_acoustic_msgs https://github.com/ros2-gbp/marine_msgs-release.git release/jazzy/marine_acoustic_msgs/2.1.0-2
clone_pinned vision_msgs https://github.com/ros2-gbp/vision_msgs-release.git release/jazzy/vision_msgs/4.1.1-3
clone_pinned gz_msgs_vendor https://github.com/ros2-gbp/gz_msgs_vendor-release.git release/jazzy/gz_msgs_vendor/0.0.8-1
clone_pinned gz_transport_vendor https://github.com/ros2-gbp/gz_transport_vendor-release.git release/jazzy/gz_transport_vendor/0.0.9-1
clone_pinned gz_tools_vendor https://github.com/ros2-gbp/gz_tools_vendor-release.git release/jazzy/gz_tools_vendor/0.0.8-1

# 기존 ROS에 없는 패키지만, test_depend를 제외한 의존성 순서로 추가 빌드한다.
# 전체 ROS 소스는 읽기만 하며 build/install/log는 BRIDGE_WS 아래에만 쓴다.
"$PYTHON" - "$ROS_ROOT/src" "$BRIDGE_WS" "$COLCON" <<'PY'
import os
import shlex
import subprocess
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_packages_with_prefixes
from catkin_pkg.condition import evaluate_condition

base, workspace, colcon = map(Path, sys.argv[1:])
installed = {name for name, prefix in get_packages_with_prefixes().items()
             if not Path(prefix).is_relative_to(workspace / 'install')}
sources = {}
# 추가 소스를 우선하며 같은 이름의 underlay 소스를 중복 전달하지 않는다.
for directory in [base, workspace / 'src']:
    result = subprocess.run([sys.executable, str(colcon), 'list', '--base-paths', str(directory)],
                            check=True, text=True, capture_output=True)
    for line in result.stdout.splitlines():
        fields = line.split('\t')
        if len(fields) >= 2:
            sources[fields[0]] = Path(fields[1]).resolve()
selected = {}
missing = set()
# ROS 패키지 누락을 시스템 라이브러리로 잘못 넘기지 않도록 rosdep 키만 허용한다.
system_dependencies = {'cmake', 'pkg-config', 'eigen', 'libsqlite3-dev', 'libzmq3-dev',
                       'protobuf-dev', 'pybind11-dev', 'python3', 'python3-dev',
                       'python3-protobuf', 'python3-psutil', 'python3-pytest',
                       'ruby', 'tinyxml2', 'uuid', 'console_bridge', 'libyaml-dev',
                       'libspdlog-dev', 'libssl-dev', 'openssl', 'asio', 'python3-numpy',
                       'python3-empy', 'python3-catkin-pkg-modules', 'python3-lark-parser',
                       'python3-yaml', 'python3-setuptools', 'python3-packaging'}
dependency_tags = {'depend', 'build_depend', 'buildtool_depend', 'build_export_depend',
                   'buildtool_export_depend', 'exec_depend'}
environment = dict(os.environ)
def collect(name):
    if name in installed or name in selected:
        return
    if name not in sources:
        # CMake, pkg-config, protobuf 같은 시스템 의존성은 Homebrew에서 제공한다.
        if name not in system_dependencies:
            missing.add(name)
        return
    folder = sources[name]
    selected[name] = folder
    for item in ET.parse(folder / 'package.xml').getroot():
        if item.tag not in dependency_tags:
            continue
        condition = item.get('condition')
        if condition and not evaluate_condition(condition, environment):
            continue
        collect(item.text.strip())
collect('ros_gz_bridge')
if missing:
    raise SystemExit('ROS 의존성 소스가 없습니다: ' + ', '.join(sorted(missing)) +
                     '\nREADME의 Jazzy 소스 가져오기 또는 SESSION3_ROS_ROOT 경로를 확인하세요.')
if not selected:
    print('ROS underlay에 ros_gz_bridge가 이미 있습니다.')
    sys.exit(0)
print('추가 빌드: ' + ', '.join(sorted(selected)), flush=True)
# macOS의 /usr/bin/make → /bin/sh 경계에서 DYLD_*가 제거된다.
# CMake가 Python을 실행하는 지점에서만 경로를 복원한다. 기존 ROS 바이너리는 건드리지 않는다.
python_launcher = workspace / 'tools' / 'python-with-ros'
python_launcher.parent.mkdir(parents=True, exist_ok=True)
library_paths = [str(workspace / 'install' / name / 'lib') for name in selected]
library_paths += os.environ.get('DYLD_LIBRARY_PATH', '').split(':')
library_path = ':'.join(dict.fromkeys(path for path in library_paths if path))
python_launcher.write_text(
    '#!/bin/sh\n'
    'export DYLD_LIBRARY_PATH=' + shlex.quote(library_path) + '\n'
    'exec ' + shlex.quote(sys.executable) + ' "$@"\n', encoding='utf-8')
python_launcher.chmod(0o755)
# 실제 빌드와 같은 보호된 셸을 거쳐 rclpy 로딩을 검사한다.
subprocess.run(['/bin/sh', '-c', shlex.join([
    str(python_launcher), '-c', 'import rclpy; import launch_ros; print("빌드용 ROS Python OK")'])],
    check=True)
command = [sys.executable, str(colcon), '--log-base', str(workspace / 'log'), 'build',
           '--base-paths', *map(str, selected.values()),
           '--build-base', str(workspace / 'build'), '--install-base', str(workspace / 'install'),
           '--symlink-install', '--parallel-workers', '2',
           '--event-handlers', 'desktop_notification-', '--packages-select', *selected,
           '--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release',
           '-DPython3_EXECUTABLE=' + str(python_launcher),
           '-DPython_EXECUTABLE=' + str(python_launcher)]
subprocess.run(command, check=True, cwd=workspace)
PY

if [[ -f "$BRIDGE_WS/install/setup.bash" ]]; then
  source "$BRIDGE_WS/install/setup.bash"
fi
ros2 pkg executables ros_gz_bridge
echo 'Mac용 Gazebo 연결 패키지 설치 완료. bash scripts/session_3.sh 로 실행하세요.'
