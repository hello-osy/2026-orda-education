# Session 2 구현

실행/장치 연결/학습/코드 읽기 사용법은 [교육 안내](../../docs/SESSION_2.md)를 보세요.

```bash
# 프로젝트 루트
bash scripts/session_2.sh  # macOS / Ubuntu / Windows WSL2
```

Windows용 가상환경이 있는 경우 PowerShell에서 `.\scripts\session_2.cmd`를 사용합니다. README의 공통 개발환경을 전제로 하며 상세한 OS별 장치 연결은 위 교육 안내를 참고하세요.

- `session_2/studio.py`: Session 1 디자인의 통합 수업 GUI
- `session_2/viewer.py`, `sensors.py`: 카메라·A1 프로세스와 실시간 표시
- `session_2/bagio.py`: 표준 ROS 2 SQLite bag 녹화, 탐색, 5프레임 추출
- `session_2/learning.py`, `jobs.py`: bbox 검증, YOLOv8 학습/추론, 작업 프로세스
- `session_2/autonomous_driving.py`: 편집기에서 처음 여는 통합 자율주행 소스
- `session_2/driving.py`: PIDNet→scan line→PID→Arduino 제어 참고 소스
- `session_2/driving_page.py`: 줄 번호·색상 강조·검색·저장을 지원하는 Python 편집기
- `scripts/`: 내부 런타임 검증 도구
- `test/`: bag, 좌표 변환, 데이터셋, 제어 정지 조건 테스트

```bash
PYTHONPATH=src/session_2:src/session_1 .venv/bin/python -m pytest src/session_2/test -q
```
