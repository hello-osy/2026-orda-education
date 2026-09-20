#!/usr/bin/env bash
# 한 번의 실행으로 ROS/프로젝트 환경을 불러오고 통합 수업 창을 연다.
set -e
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
if [[ "$(uname -s)" == Darwin ]]; then
  ROS_SETUP="$HOME/ros2_jazzy/install/setup.bash"
else
  ROS_SETUP=/opt/ros/jazzy/setup.bash
fi
if [[ ! -f "$ROS_SETUP" || ! -x .venv/bin/python || ! -f install/setup.bash ]]; then
  echo '처음 실행하기 전 README의 2. 가상환경 세팅 & ROS2 설치를 완료해 주세요.' >&2
  exit 1
fi
source "$ROS_SETUP"
source .venv/bin/activate
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 run session_1 classroom --bag "$PROJECT_DIR/rosbag2_2026_08_05-11_29_45" "$@"
