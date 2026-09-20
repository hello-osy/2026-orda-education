"""인지: SQLite rosbag의 카메라와 조향 기록을 시간 순서로 읽는다.

GUI가 재생 시간을 소유하므로 일시정지/탐색 시 별도 bag 프로세스가
뒤에서 진행되지 않는다. 기록은 읽기 전용이며 ROS 모터 토픽을 발행하지 않는다.
"""
from pathlib import Path
import bisect
import sqlite3
import cv2
import numpy as np
from .pid_view import Feedback, PID
from .scan_line_view import scan_line
from .segmentation_view import Segmenter
from .runtime import decode_image

BAG_NAME='rosbag2_2026_08_05-11_29_45'


def find_bag():
    # 실행 위치와 소스/설치 위치 양쪽에서 프로젝트의 기본 bag을 찾는다.
    for start in (Path.cwd(),Path(__file__).resolve().parent):
        for root in (start,*start.parents):
            if (root/BAG_NAME/'metadata.yaml').is_file(): return root/BAG_NAME
    raise FileNotFoundError('녹화 파일을 찾지 못했습니다. 프로젝트 폴더에서 다시 실행하세요.')


class BagFrames:
    def __init__(self,path):
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Int16
        self.connections=[];self.frames=[];feedback=[]
        path=Path(path).expanduser()
        files=sorted(path.glob('*.db3'))
        if not files: raise ValueError('이 수업에는 SQLite rosbag 폴더(.db3)가 필요합니다.')
        try:
            for file in files:
                db=sqlite3.connect(file.resolve().as_uri()+'?mode=ro',uri=True)
                self.connections.append(db)
                topics=dict(db.execute('select name,id from topics'))
                camera=topics.get('/camera/high/image_raw')
                if camera is not None:
                    self.frames.extend((stamp,len(self.connections)-1,mid) for mid,stamp in db.execute(
                        'select id,timestamp from messages where topic_id=? order by timestamp,id',(camera,)))
                fid=topics.get('/arduino/steering_raw')
                if fid is not None:
                    feedback.extend((stamp,deserialize_message(data,Int16).data) for stamp,data in db.execute(
                        'select timestamp,data from messages where topic_id=? order by timestamp,id',(fid,)))
            self.frames.sort();feedback.sort()
            if not self.frames: raise ValueError('녹화 파일에 상단 카메라 영상이 없습니다.')
            self.start=self.frames[0][0]/1e9
            self.times=[row[0]/1e9-self.start for row in self.frames]
            self.feedback_times=[];self.feedback_values=[]
            sensor=Feedback()
            for stamp,raw in feedback:
                sensor.update(raw,stamp/1e9,stamp/1e9)
                self.feedback_times.append(stamp/1e9)
                self.feedback_values.append((sensor.angle,sensor.velocity))
        except BaseException:
            self.close();raise

    def read(self,index):
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
        timestamp,connection,mid=self.frames[index]
        data=self.connections[connection].execute('select data from messages where id=?',(mid,)).fetchone()[0]
        frame=decode_image(deserialize_message(data,Image));stamp=timestamp/1e9
        pos=bisect.bisect_right(self.feedback_times,stamp)-1
        angle,velocity=(None,0.) if pos<0 or stamp-self.feedback_times[pos]>.5 else self.feedback_values[pos]
        return cv2.resize(frame,(640,352)),stamp,angle,velocity

    def close(self):
        for db in self.connections: db.close()
        self.connections=[]


class LessonEngine:
    """판단과 제어: 모델 → 차선 위치 → 목표각 → PID. GUI와 독립적으로 검증한다."""
    def __init__(self,bag,device='auto'):
        self.bag=BagFrames(bag)
        try: self.segmenter=Segmenter(device=device)
        except BaseException: self.bag.close();raise
        self.pid=PID();self.p_only=PID(6.5,0.,0.);self.cached_index=None;self.last_index=None;self.cached=None

    def process(self,index,settings,reset=False):
        if index!=self.cached_index:
            frame,stamp,angle,velocity=self.bag.read(index)
            self.cached=(frame,stamp,angle,velocity,self.segmenter.predict(frame))
            self.cached_index=index
        frame,stamp,angle,velocity,labels=self.cached
        # 탐색/되감기/설정 변경은 과거 적분 상태를 이어 쓰지 않는다.
        if reset or self.last_index is None or index<=self.last_index:
            self.pid.reset();self.p_only.reset()
        self.pid.kp,self.pid.ki,self.pid.kd=settings['gains']
        self.p_only.kp=self.pid.kp
        scan=scan_line(labels,settings['scan_y'],settings['target_x'])
        target=None if scan.error_px is None else float(np.clip(-scan.error_px/130*45,-45,45))
        # 비교용 P도 같은 입력과 PWM 제한/최소 출력/허용 오차를 적용한다.
        result=self.pid.update(target,angle,velocity,stamp)
        p_result=self.p_only.update(target,angle,velocity,stamp)
        self.last_index=index
        return dict(index=index,frame=frame,labels=labels,scan=scan,pid=result,p_only=p_result,gains=tuple(settings['gains']))

    def close(self): self.bag.close()
