# Session 3

[수업 명령어](../../docs/SESSION_3.md) · [검증 범위](VALIDATION.md)

추가 설치 후 프로젝트 루트에서 `bash scripts/session_3.sh`를 실행합니다.

- 시뮬레이터: Gazebo Harmonic, 앞바퀴 조향, 흰 실선 타원 트랙, 카메라 2개와 라이다.
- 실차: Session 2의 USB 센서·Arduino 규격을 사용하며 구동 PWM은 항상 0입니다.
- 조향: PWM ±150의 짧은 펄스 후 새 피드백을 기다립니다. 목표 1도 이내 도달·통과 시 정지하며 같은 목표는 재시작하지 않습니다. 목표 처리 시간은 최대 5초입니다.
- 중앙: 펌웨어의 초기 중앙 맞춤을 기다린 뒤 안정된 피드백으로 자동 등록합니다. 기본 `arm:=false`는 ROS 제어 출력을 차단합니다.
- 영상: 실차 `bgr8`, Gazebo `rgb8`, 기본 320×240·10 Hz. 시각화는 교육자료의 `yuv422_yuy2`도 처리합니다.
- 라이다: 앞 0°, 왼쪽 +90°, 거리 m. Session 2의 과거 bag 각도 기준과 다릅니다.

```bash
source scripts/session_3_env.sh 30
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/session_3/test -q
```
