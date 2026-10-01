#!/usr/bin/env bash
# 프로젝트 가상환경으로 2회차 수업을 실행한다. ROS 환경은 필요하지 않다.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
if [[ ! -x .venv/bin/python ]]; then
  echo 'README.md의 가상환경 설치를 먼저 완료해 주세요.' >&2
  exit 1
fi
exec "$PROJECT_DIR/.venv/bin/python" "$PROJECT_DIR/scripts/session_2.py" "$@"
