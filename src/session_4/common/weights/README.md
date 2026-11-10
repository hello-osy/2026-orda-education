# 정답 YOLO 모델

`traffic_cone_yolov8n.pt`는 제공된 Session 4 Gazebo 맵의 카메라 영상으로 학습한 모델입니다.

- 종류: `red`, `yellow`, `green`, `cone`
- 학습 210장 / 검증 59장, YOLOv8n, 320px, 40 epoch, Apple MPS
- 라벨은 시뮬레이터 색 마스크로 만들고 일부 겹쳐 그린 영상을 확인했습니다. 검증 수치는 이 임시 라벨과의 일치도이며 실차 성능을 뜻하지 않습니다.
- 원본·학습 로그: 프로젝트 `workspace/session_4/reference_dataset`, `reference_training`
- 가중치 해시·학습 조건: `training_info.json`
- 초기 가중치·학습 도구: [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/)

시뮬레이터를 켜고 `python3 src/session_4/student/driving.py --solution`을 실행하면 이 모델과 시각화 창을 함께 사용합니다.
