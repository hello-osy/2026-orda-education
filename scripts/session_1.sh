#!/usr/bin/env bash
# 프로젝트 위치에 맞춰 ROS 환경을 불러오고 1회차 수업을 실행한다.
set -e
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if [[ "$(uname -s)" == Darwin ]]; then
  ROS_SETUP="$HOME/ros2_jazzy/install/setup.bash"
else
  ROS_SETUP=/opt/ros/jazzy/setup.bash
fi
if [[ ! -f "$ROS_SETUP" || ! -x .venv/bin/python || ! -f install/setup.bash ]]; then
  echo 'README.md의 공통 설치와 Session 1 빌드를 먼저 완료해 주세요.' >&2
  exit 1
fi
source "$ROS_SETUP"
source .venv/bin/activate
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 run session_1 classroom --bag "$PROJECT_DIR/rosbag2_2026_08_05-11_29_45" "$@"
