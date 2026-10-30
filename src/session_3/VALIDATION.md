# Session 3 검증 (2026-10-30)

`Ctrl+C` 종료 지연도 별도로 검사했습니다. 실제 멈춘 bag 재생 프로세스는 키보드 스레드의 `read()` / `join()`에서 대기했습니다. 격리 도메인·가상 터미널에서도 기본 녹화·재생은 5초 안에 종료되지 않았고 Enter 입력 후 약 0.17초에 종료됐습니다. `--disable-keyboard-controls` 적용 후 녹화 0.149초·재생 0.258초, 일반 `ros2 topic pub`는 0.170초에 종료됐습니다. 측정 결과는 `workspace/session_3/validation/cli_shutdown/summary.json`과 같은 폴더의 로그에 저장했습니다. 이는 해당 Mac 환경에서의 측정값입니다.

macOS 27.0.1 · Python 3.10 · 기존 소스 빌드 ROS 2 Jazzy에서 확인했습니다. Gazebo Sim 8.15, msgs 10.4, transport 13.6을 사용했습니다.

| 항목 | 결과 |
|---|---|
| colcon 빌드 / 실행 항목 6개 / launch 3개 로딩 | 통과 |
| 단위·센서 ROS 메시지 테스트 | 최종 구현 58개 통과 |
| 카메라·라이다 GUI | 별도 ROS 프로세스의 합성 데이터 수신, 중단 표시와 화면 확인 |
| 실차 제어 코드 | 실제 ROS + 가상 시리얼 PTY 20개 검사 통과; 송신 355개 모두 구동 PWM 0, 36개 펄스의 최대 폭 23.3ms |
| bag | 실제 CLI로 SQLite 녹화(high 15 / low 15 / scan 16), 별도 도메인 재생·시계 수신 확인 |
| 시뮬레이터 제어 노드 | 실제 ROS로 속도 변환, 잘못된 입력 정지, `/clock` 없이 watchdog 작동 확인 |
| Gazebo 실제 센서 | high/low 영상·CameraInfo·360방향 라이다·`/clock` 수신; 수정한 트랙 메시의 도로·흰 실선 영상 확인 |
| Gazebo 실제 물리 주행 | 2회 통과; 최근 실행에서 직진 0.3066m, 좌회전 +0.0808rad, 명시 정지·watchdog 정지 후 위치 변화 0m |
| macOS C++ bridge | 의존성을 포함한 9개 패키지 빌드 완료(2분 2초), 실제 Gazebo↔ROS 메시지 교환 확인 |
| Gazebo 3D GUI | GUI 프로세스 실행 확인; 실제 3D 창의 화면 검수는 아직 하지 않음 |
| USB 센서 동시 수신 | 실물 C920 2대·라이다·Mega를 연결한 35초 검사 통과(`arm:=false`) |
| 실제 조향 | 단독·센서 동시 실행에서 +5→0→−5→0 목표의 1도 이내 진입 후 출력 0 유지 확인; 정지 후 최종 위치 오차 1.5도 조건은 동시 검사에서 실패 |

물리 주행값은 Gazebo의 실제 world pose에서 계산했습니다. 바퀴 회전으로 적분한 odometry 값이 아닙니다. 최근 결과는 `workspace/session_3/validation/sim_runtime_s3e97y1e/result.json`이며, high/low 각 114장·라이다 71개·시계 6690개를 수신했습니다. `output/session3-validation/gazebo-high-verified.png`와 `gazebo-low-verified.png`에서 어두운 도로와 흰 실선을 확인했습니다.

macOS bridge 빌드는 CMake 3 및 Python 실행 래퍼를 사용합니다. 보호된 `/bin/sh`를 거치며 사라지는 `DYLD` 경로를 Python 실행 직전에 복구해 ROS 공유 라이브러리를 찾도록 했습니다.

실물 센서 검사는 `workspace/session_3/validation/vehicle_local/result.json`에 저장했습니다. C920 카메라는 각각 347장(320×240, 9.91Hz), 라이다는 246개 스캔(7.03Hz, 360방향), Mega 조향 피드백은 701개(20Hz, raw 501~502)를 수신했습니다. 마지막 스캔의 유효 거리는 146방향이었고 `/motor_control` 구독자도 확인했습니다. 이 검사는 모터 출력을 비활성화한 센서 통신 검사이며 실제 조향 성공을 뜻하지 않습니다.

현재 구현은 ±150을 20ms 출력한 뒤 0으로 멈추고, 150ms 안정 시간 이후 새로운 시리얼 피드백을 기다립니다. 목표 도달·통과 시 정지를 유지하고 같은 목표는 재시작하지 않으며 목표 처리 시간은 최대 5초입니다. 실물 단독 검사에서는 +5→0→−5→0 목표에 대해 센서 추정각 +5.625→0→−4.5→−0.5625도와 출력 0 유지가 확인됐습니다(`vehicle_local/steering_150_pulse_result.json`). 각 단계는 약 1.3~2.7초였습니다. 이 값은 `raw_span=80` 가정으로 계산한 값이며 외부 장비로 실제 바퀴 각도를 측정한 결과가 아닙니다.

센서를 함께 실행한 검사도 네 목표 모두 1도 이내에 진입한 뒤 출력을 0으로 유지했습니다. 마지막 중앙 복귀는 추정각 −0.5625도에서 정지한 뒤 −1.6875도로 변했습니다. 따라서 **도달 후 정지·재시작 차단은 확인됐지만, 정지 후 위치 오차 1.5도 이내 조건은 통과하지 못했습니다.** 실패한 [원본 결과](../../workspace/session_3/validation/vehicle_local/combined_steering_result.json)와 전체 추적을 기준으로 정지 동작을 확인한 [검토 결과](../../workspace/session_3/validation/vehicle_local/combined_stop_review.json)를 함께 보존했습니다. 정지 후 위치를 능동적으로 유지하는 제어는 하지 않습니다.

동시 관찰 약 8초 동안 카메라 각 80장(약 9.8Hz), 라이다 57개(7.05Hz), 조향 피드백 161개(약 20Hz)가 수신됐습니다. 조향 검사기가 약 14.8초에 launch를 종료해 35초 센서 모니터의 최종 결과는 `false`입니다. 이는 종료 이후의 미수신을 포함한 결과이며 센서 단독 35초 검사 통과와 구분합니다.

펄스 길이 결정 과정에서 연속 PWM 150은 +5도 목표를 센서 추정각 14.625도로 지나쳐 실패했고 정지했습니다(`vehicle_local/steering_150_result.json`). 10ms 펄스는 목표에 도달하지 못하고 5초 제한으로 정지해, 현재 20ms를 사용합니다.

자동 중앙 등록은 10초 대기 후 2초간 안정된 피드백(범위 3 이하, 시리얼 표본 8개 이상)을 요구합니다. 가장 빠른 등록 시점은 약 12초입니다. 펌웨어가 전원 연결 시 중앙을 찾는다는 동작은 사용자 설명을 따릅니다. 알려진 펌웨어 완료 신호가 없어, 센서 안정성은 중앙 맞춤 완료의 추정 조건이며 완료를 직접 확인한 신호가 아닙니다. 등록 중앙은 `/steering/center`, 마지막으로 시리얼 송신에 성공한 조향 PWM은 `/steering/pwm`으로 발행합니다.

**미검증:** Ubuntu/WSL, 다른 PC와의 LAN 통신, 실제 조향각의 외부 계측은 별도 검증이 필요합니다. 합성 센서 GUI 검사·Gazebo 센서 렌더링·실물 USB 센서 검사는 구분했습니다.

검사 종료 후 실행 프로세스와 USB 포트 점유가 모두 해제된 것을 확인했습니다.

```bash
source scripts/session_3_env.sh 30
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/session_3/test -q
python src/session_3/scripts/check_motor_runtime.py
python src/session_3/scripts/check_ros_runtime.py
python src/session_3/scripts/check_sim_runtime.py
```

위 재현 명령은 실제 USB 장치를 사용하지 않습니다. 증거는 `workspace/session_3/validation/`, GUI 확인 화면은 `output/session3-validation/`에 저장했습니다. `PYTEST_DISABLE_PLUGIN_AUTOLOAD`는 기존 ROS 테스트 플러그인과 새 pytest의 충돌을 피합니다.

구현 참고: [Gazebo Harmonic 설치](https://gazebosim.org/docs/harmonic/install/), [AckermannSteering](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/ackermann_steering/AckermannSteering.cc), [ROS bridge 1.0.25](https://github.com/gazebosim/ros_gz/tree/1.0.25).
