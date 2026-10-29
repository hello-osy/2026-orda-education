#!/usr/bin/env bash
# 인자 없이 실행: 환경 → session_3 빌드 → Gazebo.
set -e
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if [ "$#" -ne 0 ]; then
  echo '사용법: bash scripts/session_3.sh (뒤에 인자를 붙이지 않습니다)' >&2
  exit 2
fi
source scripts/session_3_env.sh 30
python src/session_3/scripts/check_install.py --sim
touch .venv/COLCON_IGNORE
python -m colcon build --symlink-install --packages-select session_3 --event-handlers desktop_notification-
source install/local_setup.bash
exec python "$(command -v ros2)" launch session_3 simulator.launch.py
