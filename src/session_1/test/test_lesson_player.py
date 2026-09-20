"""시간 탐색이 미래의 피드백을 쓰지 않고 PID 상태를 안전하게 초기화하는지 검증."""
import sqlite3
import numpy as np
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from session_1.lesson_player import BagFrames,LessonEngine

@pytest.fixture
def bag(tmp_path):
    db=sqlite3.connect(tmp_path/'sample.db3')
    db.executescript('CREATE TABLE topics(id INTEGER,name TEXT); CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    db.executemany('INSERT INTO topics VALUES (?,?)',[(1,'/camera/high/image_raw'),(2,'/arduino/steering_raw')])
    frame=Image(height=2,width=4,encoding='bgr8',step=12,data=bytes([70]*24))
    rows=[(1,1,1_000_000_000,serialize_message(frame)),
          (2,2,1_100_000_000,serialize_message(Int16(data=500))),
          (3,1,1_200_000_000,serialize_message(frame)),
          (4,2,1_300_000_000,serialize_message(Int16(data=480))),
          (5,1,1_400_000_000,serialize_message(frame)),
          (6,1,2_100_000_000,serialize_message(frame))]
    db.executemany('INSERT INTO messages VALUES (?,?,?,?)',rows);db.commit();db.close();return tmp_path


def test_recorded_feedback_seek_and_staleness(bag):
    reader=BagFrames(bag)
    try:
        assert reader.read(0)[2] is None  # 미래 피드백을 미리 사용하지 않는다
        assert reader.read(1)[2]==0.
        assert reader.read(2)[2]==11.25
        assert reader.read(3)[2] is None  # 0.5초 넘게 지난 값은 사용하지 않는다
        assert reader.read(1)[2]==0.      # 되감아도 동일한 기록을 사용한다
        assert reader.read(0)[0].shape==(352,640,3)
    finally:reader.close()


def test_engine_reuses_model_and_resets_on_seek(bag,monkeypatch):
    class Model:
        calls=0
        def __init__(self,device):pass
        def predict(self,frame):
            self.calls+=1;labels=np.zeros(frame.shape[:2],np.uint8);labels[:,530:540]=2;return labels
    monkeypatch.setattr('session_1.lesson_player.Segmenter',Model)
    engine=LessonEngine(bag,'cpu');settings={'gains':(6.5,1.,.8),'scan_y':.75,'target_x':.79}
    try:
        initial=engine.process(1,settings,True)
        engine.process(2,settings)
        repeated=engine.process(2,settings,True)
        assert engine.segmenter.calls==2  # 같은 영상의 설정 변경은 모델 추론을 재사용한다
        returned=engine.process(1,settings,True)
        assert initial['pid']==returned['pid']
        assert abs(repeated['pid'].output)<=150
        assert engine.process(0,settings,True)['pid'].output==0
    finally:engine.close()


def test_missing_bag_is_actionable(tmp_path):
    with pytest.raises(ValueError,match='SQLite rosbag'):BagFrames(tmp_path)


def test_p_and_pid_comparison_uses_same_input_and_limits(bag,monkeypatch):
    class Model:
        def __init__(self,device):pass
        def predict(self,frame):
            labels=np.zeros(frame.shape[:2],np.uint8);labels[:,530:540]=2;return labels
    monkeypatch.setattr('session_1.lesson_player.Segmenter',Model)
    engine=LessonEngine(bag,'cpu')
    settings={'gains':(6.5,0.,0.),'scan_y':.75,'target_x':.79}
    try:
        for index in (0,1,2,3):
            data=engine.process(index,settings,True)
            assert data['p_only']==data['pid']  # I=D=0이면 계산과 출력 제약이 모두 같아야 한다
        settings['gains']=(6.5,0.,.8)
        settings['target_x']=.82  # ±1° 정지 구간 밖에서 D 보정 비교
        data=engine.process(2,settings,True)
        assert data['p_only'].target==data['pid'].target
        assert data['p_only'].angle==data['pid'].angle
        assert data['p_only'].p==data['pid'].p
        assert data['p_only'].d==0 and data['pid'].d!=0
        assert data['p_only'].output!=data['pid'].output
        settings['gains']=(20.,5.,5.)
        data=engine.process(2,settings,True)
        for result in (data['p_only'],data['pid']):
            assert abs(result.output)<=150
            assert result.output==0 or abs(result.output)>=40
        missing=engine.process(3,settings,True)
        assert missing['p_only'].output==missing['pid'].output==0
    finally:engine.close()
