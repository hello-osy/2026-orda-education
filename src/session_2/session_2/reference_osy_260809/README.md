# 4번 스케일카 주행 코드 출처

- 저장소: https://github.com/hello-osy/ai-autonomous-driving-competition-2026
- 사용자 지정 브랜치: `osy-260809`
- 고정 커밋: `3a29efdae1467355c1d7f059f92238febf85a33a`
- `.py.txt` 파일은 해당 커밋의 원문이며 실행 시 ROS를 import하지 않도록 자료로 보관한다.

`competition_lane.py`는 timed_lane_offset_node의 상수, BEV, 마스크 필터, 차선 선택과 상태 전이 계산을 이식했다. ROS 토픽/로그는 GUI 반환값으로 바꾸었으며 알고리즘 비교 테스트는 원문을 직접 실행해 같은 입력 시퀀스의 결과를 비교한다. `seg_lane_offset_node`의 클래스 색 매핑과 3회 워밍업, `timed_lane_main_node`의 출발 1초 직진은 `competition_driving.py`가 담당한다.

PID 계산과 Serial 프로토콜은 기존 어댑터를 재사용하며 원본 timed 기본값과 같다. 실제 피드백은 원본처럼 median 없이 raw를 변환하고 각속도 EMA를 적용한다. GUI의 전진 PWM 150~230, ramp, 수동 중앙 보정, 사용자 arm 및 stale 입력 정지는 유지한다. ROS 노드 실행과 자동 출발, PWM 255는 가져오지 않는다.

기존 Session 1 PIDNet 가중치와 이 커밋의 `lane_seg/models/lane_pidnet_s.pt`는 같은 Git blob `94d83ad0ae57fd87257dcdfe54ac9718c3c7236d`이다. 클래스 순서는 background, road, lane_solid, lane_dashed, green_mat, light_gray이다. 원본 camera.yaml의 640×360 입력 크기는 4번 파이프라인 내부에서 맞춘다.

1·2·3번 및 `autonomous_driving.py`는 이 파이프라인을 사용하지 않는다.

## 검증 (2026-10-06)

- 원문과 이식 코드에 동일한 합성 영상 시퀀스를 넣어 RIGHT 복귀, CENTER FALLBACK, CENTER JUMP HOLD, CENTER LOST HOLD의 목표각·상태·기준 위치·안정화 카운트 일치 확인.
- 4번 전용/GUI/모터/기존 교육 코드 회귀 검사 47개 통과. 출발 1초 직진, 영상 끊김 정지, 명시적 시작, 단계 이동 정지 및 모의 시리얼 종료 명령 포함.
- 실제 두 C920 + MPS + 학습된 YOLO 모델을 4번 GUI에서 실행해 차선 결과 40프레임 확인. CENTER FALLBACK과 RIGHT OK 전환 확인, 최대 결과 나이 0.203초 미만, 두 카메라 오류 없음, YOLO 마지막 처리 16ms, 정상 종료.
- 모터 포트는 열지 않았다. 바퀴 조향·실제 트랙 주행은 검증하지 않았다.
