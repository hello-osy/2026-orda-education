#!/usr/bin/env bash
# 기존 수업처럼 프로젝트 .venv / colcon / ROS launch를 사용한다.
set -e
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
action="${1:-sim}"
[ "$#" -eq 0 ] || shift
case "$action" in
  sim|manual|collect|record|extract|train|starter|starter-yolo|solution|solution-yolo|yolo|viewers|debug|build) ;;
  *) echo '사용법: bash scripts/session_4.sh [sim|collect|manual|record|extract|train|starter|starter-yolo|solution|solution-yolo|debug|viewers|build]' >&2; exit 2 ;;
esac
source scripts/session_4_env.sh
if [ "$action" = sim ]; then
  python src/session_4/common/tools/check_install.py --sim
else
  python src/session_4/common/tools/check_install.py
fi
touch .venv/COLCON_IGNORE
python src/session_4/common/tools/repair_install.py
# 시뮬레이터를 켠 뒤 manual/collect 등을 실행할 때는 모델 경로를 건드리지 않습니다.
# 학생 driving.py와 공통 Python 코드는 원본 파일에 연결되어 바로 반영됩니다.
if [ "$action" = sim ] || [ "$action" = build ] || [ ! -f install/session_4/share/session_4/package.xml ]; then
  python -m colcon build --symlink-install --packages-select session_3 session_4 --event-handlers desktop_notification-
fi
python src/session_4/common/tools/repair_install.py --after-build
source install/local_setup.bash
case "$action" in
  sim) exec python "$(command -v ros2)" launch session_4 simulator.launch.py "$@" ;;
  manual) exec python "$(command -v ros2)" run session_4 manual_drive "$@" ;;
  collect) exec python -m session_4.recording "$@" ;;
  record) exec python -m session_4.recording --record-only "$@" ;;
  extract) exec python -m session_4.dataset_tools "$@" ;;
  train) exec python -m session_4.training_tools "$@" ;;
  starter) exec python src/session_4/student/driving.py "$@" ;;
  starter-yolo) exec python src/session_4/student/driving.py --yolo "$@" ;;
  solution) exec python src/session_4/student/driving.py --solution "$@" ;;
  solution-yolo) exec python src/session_4/student/driving.py --solution --yolo "$@" ;;
  yolo) exec python "$(command -v ros2)" run session_4 yolo_node --ros-args -p detector:=yolo "$@" ;;
  viewers) exec python "$(command -v ros2)" launch session_3 viewers.launch.py "$@" ;;
  debug) exec python "$(command -v ros2)" run session_4 debug_viewer "$@" ;;
  build) ;;
esac
