"""공통 입출력: 인지(ROS 카메라 수신), 판단 파이프라인, 제어 결과 시각화.

세 수업 파일은 독립 실행한다. 한 프레임의 원본과 결과를 함께 표시한다.
ROS 없이 이미지/영상으로도 같은 판단 코드를 실행할 수 있다.
"""
import argparse
import json
import time
from pathlib import Path
import cv2
import numpy as np
from .segmentation_view import Segmenter
from . import segmentation_view, scan_line_view, pid_view
from .drawing import text
from . import reference_color_filter_view, reference_sliding_window_view, pipeline_after_view


def decode_image(msg, compressed=False):
    """인지: ROS Image를 BGR로 변환. step 패딩과 실제 YUY2 카메라 지원."""
    if compressed:
        frame=cv2.imdecode(np.frombuffer(bytes(msg.data),np.uint8),cv2.IMREAD_COLOR)
        if frame is None: raise ValueError('압축 영상 디코딩 실패')
        return frame
    if msg.width<=0 or msg.height<=0 or msg.step<=0:
        raise ValueError('Image 크기와 step은 양수여야 합니다')
    enc=msg.encoding.lower()
    channels={'bgr8':3,'rgb8':3,'mono8':1,'bgra8':4,'rgba8':4,
              'yuv422_yuy2':2,'yuyv':2,'yuy2':2,'yuv422':2,'uyvy':2}
    if enc not in channels: raise ValueError(f'지원하지 않는 encoding: {enc}')
    n=channels[enc]
    if n==2 and msg.width%2:
        raise ValueError('YUV422 영상 폭은 짝수여야 합니다')
    if msg.step < msg.width*n: raise ValueError('Image.step이 width보다 작습니다')
    raw=np.frombuffer(bytes(msg.data),np.uint8)
    if raw.size != msg.height*msg.step: raise ValueError('Image 데이터 길이 불일치')
    arr=raw.reshape(msg.height,msg.step)[:,:msg.width*n].reshape(msg.height,msg.width,n).copy()
    codes={'rgb8':cv2.COLOR_RGB2BGR,'mono8':cv2.COLOR_GRAY2BGR,
           'bgra8':cv2.COLOR_BGRA2BGR,'rgba8':cv2.COLOR_RGBA2BGR,
           'yuv422_yuy2':cv2.COLOR_YUV2BGR_YUY2,'yuyv':cv2.COLOR_YUV2BGR_YUY2,
           'yuy2':cv2.COLOR_YUV2BGR_YUY2,'yuv422':cv2.COLOR_YUV2BGR_UYVY,'uyvy':cv2.COLOR_YUV2BGR_UYVY}
    return cv2.cvtColor(arr,codes[enc]) if enc in codes else arr


class Pipeline:
    def __init__(self,stage,args):
        self.stage,self.args=stage,args
        self.segmenter=None if stage=='reference_color_filter' else Segmenter(args.model,args.device)
        self.color_filter=reference_color_filter_view.ColorFilterView() if stage=='reference_color_filter' else None
        self.pid=pid_view.PID(args.kp,args.ki,args.kd)
        self.history=pid_view.History()
        self.feedback=pid_view.Feedback()
        self.plant=pid_view.DemoPlant()
        self.last_stamp=None
        self.count=0

    def process(self,frame,stamp):
        started=time.perf_counter()
        # 판단 1 / segmentation: 종횡비를 유지하고 모델 입력은 32배수로 맞춘다.
        height=max(64,round(frame.shape[0]*self.args.width/frame.shape[1]/32)*32)
        frame=cv2.resize(frame,(self.args.width,height))
        labels=self.segmenter.predict(frame) if self.segmenter else None
        metrics={'stage':self.stage,'stamp':float(stamp),'device':self.segmenter.device if self.segmenter else 'CPU color filter',
                 'width':frame.shape[1],'height':frame.shape[0]}
        if self.stage=='reference_color_filter':
            canvas,details=self.color_filter.render(frame)
            metrics['filters']=details
        elif self.stage=='reference_sliding_window':
            canvas,details=reference_sliding_window_view.visualize(
                frame,labels,self.args.lane,self.args.windows,self.args.margin,self.args.min_pixels)
            metrics.update(details)
        elif self.stage=='segmentation':
            canvas=segmentation_view.visualize(frame,labels)
        else:
            # 판단 2 / scan line: segmentation 결과를 그대로 사용한다.
            scan=scan_line_view.scan_line(labels,self.args.scan_y,self.args.target_x,self.args.lane,self.args.band)
            metrics.update(target_x=scan.target_x,measured_x=scan.measured_x,error_px=scan.error_px)
            if self.stage=='scan_line': canvas=scan_line_view.visualize(frame,labels,scan)
            else:
                # 제어 / PID: 프레임 획득 시각으로 dt 계산. 재생 속도와 분리된다.
                if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>1.):
                    self.history.values.clear()
                target=None if scan.error_px is None else float(np.clip(
                    self.args.steering_sign*scan.error_px/(130*self.args.width/640)*45,-45,45))
                if self.args.feedback=='demo':
                    angle,velocity=self.plant.step(stamp)
                else:
                    fresh=self.feedback.received is not None and time.monotonic()-self.feedback.received<=.5
                    angle=self.feedback.angle if fresh else None
                    velocity=self.feedback.velocity
                result=self.pid.update(target,angle,velocity,stamp)
                self.plant.pwm=result.output
                self.history.add(scan.error_px,result)
                renderer=pipeline_after_view if self.stage=='pipeline_after' else pid_view
                canvas=renderer.visualize(frame,labels,scan,result,self.history,
                                          "DEMO simulated" if self.args.feedback=="demo" else "RECORDED A1")
                metrics.update(p=result.p,i=result.i,d=result.d,target_angle_deg=result.target,actual_angle_deg=angle,
                               angle_error_deg=result.error,steering_pwm=result.output,feedback=self.args.feedback,
                               raw_pwm=result.raw,dt=result.dt,status=result.status)
        self.last_stamp=stamp;self.count+=1
        metrics['processing_ms']=(time.perf_counter()-started)*1000
        footer=np.full((32,canvas.shape[1],3),24,np.uint8)
        text(footer,f"{metrics['processing_ms']:.1f} ms | {metrics['device']} | q: quit | space: pause | r: reset PID | visualization only",(10,21))
        return np.vstack([canvas,footer]),metrics


def parser(stage):
    p=argparse.ArgumentParser(description=f'ORDA {stage}: BEFORE / AFTER')
    p.add_argument('--source',choices=['ros','image','video'],default='ros')
    p.add_argument('--input',default='',help='이미지 또는 동영상 경로')
    p.add_argument('--topic',default='/camera/high/image_raw')
    p.add_argument('--compressed',action='store_true',help='CompressedImage 토픽')
    p.add_argument('--model',default='',help='dataset_info.json과 함께 있는 가중치 경로')
    p.add_argument('--device',choices=['auto','cpu','mps','cuda'],default='auto')
    p.add_argument('--width',type=int,default=640)
    p.add_argument('--scan-y',type=float,default=.75,help='scan line y 비율 0~1')
    p.add_argument('--target-x',type=float,default=.79,help='선이 있어야 할 목표 x 비율 0~1')
    p.add_argument('--lane',choices=['right','center'],default='right')
    p.add_argument('--band',type=int,default=5)
    p.add_argument('--kp',type=float,default=6.5)
    p.add_argument('--ki',type=float,default=0.)
    p.add_argument('--kd',type=float,default=.8)
    p.add_argument('--steering-sign',type=float,choices=[-1.,1.],default=-1.)
    p.add_argument('--feedback',choices=['auto','ros','demo'],default='auto',help='auto: ROS는 기록된 A1, image/video는 모의 조향')
    p.add_argument('--feedback-topic',default='/arduino/steering_raw')
    p.add_argument('--headless',action='store_true',help='GUI 없이 교육 토픽/PNG만 출력')
    p.add_argument('--image-rate',type=float,default=5.,help='큰 시각화 Image 토픽 최대 발행 Hz. 화면 처리율과 별개')
    p.add_argument('--output',default='',help='마지막 비교 화면 PNG 저장 경로')
    p.add_argument('--max-frames',type=int,default=0,help='0이면 입력 끝 또는 종료까지')
    p.add_argument('--windows',type=int,default=9,help='참고 sliding window 개수')
    p.add_argument('--margin',type=int,default=50,help='참고 검색창 좌우 폭 px')
    p.add_argument('--min-pixels',type=int,default=20,help='참고 창 재정렬 최소 픽셀 수')
    p.add_argument('--fps',type=float,default=30.,help='영상에 FPS 메타데이터가 없을 때')
    return p


def make_gui(title,color_filter=None):
    # headless 모드에서는 Qt를 import하지 않는다.
    from .gui import Viewer
    return Viewer(title,color_filter)


def run(stage,argv=None):
    p=parser(stage);args,ros_args=p.parse_known_args(argv)
    if args.source!='ros' and ros_args: p.error(f'알 수 없는 옵션: {ros_args}')
    if args.width<320 or args.width%32: p.error('--width는 320 이상 32의 배수')
    if not 0<=args.scan_y<=1 or not 0<=args.target_x<=1 or args.band<1: p.error('scan 비율은 0~1, band>=1')
    if args.fps<=0 or not np.isfinite(args.fps) or args.max_frames<0: p.error('fps>0, max-frames>=0')
    if not np.isfinite(args.image_rate) or not 0<args.image_rate<=30: p.error('0 < image-rate <= 30')
    if args.feedback=='auto': args.feedback='ros' if args.source=='ros' else 'demo'
    if args.feedback=='ros' and args.source!='ros': p.error('--feedback ros는 --source ros에서만 사용')
    if args.windows<3 or args.windows>64 or args.margin<1 or args.min_pixels<1: p.error('3<=windows<=64, margin/min-pixels>=1')
    pipeline=Pipeline(stage,args)
    window=f'ORDA session_1 / {stage}'
    gui=None if args.headless else make_gui(window,pipeline.color_filter)
    latest=None;paused=False;last_input=None

    def display(frame,stamp):
        nonlocal latest,last_input
        last_input=(frame,stamp)
        latest,metrics=pipeline.process(frame,stamp)
        if gui is not None: gui.show_frame(latest,metrics,pipeline.count)
        if pipeline.count==1: print(json.dumps(metrics,ensure_ascii=False),flush=True)
        return latest,metrics

    def key():
        nonlocal paused
        if args.headless: return True
        k=gui.poll()
        if k in (27,ord('q')): return False
        if k==ord(' '):
            paused=not paused
            gui.set_paused(paused)
        if k==ord('r'):
            pipeline.pid.reset();pipeline.history.values.clear();gui.mark_reset()
        # 새 카메라 프레임이 없어도 슬라이더 변경을 마지막 프레임에 반영한다.
        if pipeline.color_filter and pipeline.color_filter.dirty and last_input is not None:
            display(*last_input)
        return True

    node=None;cap=None;rclpy=None
    def steps():
        nonlocal node,cap,rclpy
        try:
            if args.source=='ros':
                import rclpy
                from rclpy.node import Node
                from rclpy.executors import ExternalShutdownException
                from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
                from sensor_msgs.msg import Image,CompressedImage
                from std_msgs.msg import String,Int16
                rclpy.init(args=ros_args)
                node=Node(f'{stage}_view')
                qos=QoSProfile(depth=1,history=HistoryPolicy.KEEP_LAST,reliability=ReliabilityPolicy.BEST_EFFORT)
                pending=[None]
                if stage in ('pid','pipeline_after') and args.feedback=='ros':
                    def receive_feedback(msg):
                        pipeline.feedback.update(int(msg.data),node.get_clock().now().nanoseconds/1e9,time.monotonic())
                    feedback_sub=node.create_subscription(Int16,args.feedback_topic,receive_feedback,qos)
                # 인지: 최신 메시지 하나만 보관. 추론 지연이 과거 영상 큐로 누적되지 않는다.
                def receive(msg): pending[0]=msg
                subscription=node.create_subscription(CompressedImage if args.compressed else Image,args.topic,receive,qos)
                image_pub=node.create_publisher(Image,f'/session_1/{stage}/image',1)
                values_pub=node.create_publisher(String,f'/session_1/{stage}/metrics',1)
                node.get_logger().info(f'대기 중: {args.topic}. ros2 bag play <경로> --clock 실행')
                last_image_publish=0.
                while rclpy.ok():
                    yield
                    if not key(): break
                    try: rclpy.spin_once(node,timeout_sec=.01)
                    except ExternalShutdownException: break
                    if paused or pending[0] is None: continue
                    msg=pending[0];pending[0]=None
                    try: frame=decode_image(msg,args.compressed)
                    except (ValueError,cv2.error) as exc:
                        node.get_logger().error(str(exc));continue
                    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                    # 0 타임스탬프는 수신 시각을 사용. 정상 bag에서는 header 시각을 유지한다.
                    if stamp==0: stamp=time.monotonic()
                    canvas,metrics=display(frame,stamp)
                    # 제어/출력: 교육용 화면과 수치만 발행. 모터 토픽 발행자는 없다.
                    now=time.monotonic()
                    # 2x2 컬러 화면은 수 MB: 빠른 재생에서 DDS 이미지 큐가 포화되지 않게 제한.
                    if now-last_image_publish>=1./args.image_rate:
                        out=Image();out.header=msg.header
                        out.height,out.width=canvas.shape[:2];out.encoding='bgr8';out.step=out.width*3
                        out.data=canvas.tobytes();image_pub.publish(out)
                        last_image_publish=now
                    values=String();values.data=json.dumps(metrics,allow_nan=False);values_pub.publish(values)
                    if args.max_frames and pipeline.count>=args.max_frames: break
            else:
                if not args.input: p.error('--source image/video에는 --input이 필요합니다')
                if args.source=='image':
                    frame=cv2.imread(str(Path(args.input).expanduser()))
                    if frame is None: raise ValueError(f'이미지를 읽지 못했습니다: {args.input}')
                    display(frame,0.)
                    while gui is not None:
                        yield
                        if not key(): break
                else:
                    cap=cv2.VideoCapture(str(Path(args.input).expanduser()))
                    if not cap.isOpened(): raise ValueError(f'영상을 읽지 못했습니다: {args.input}')
                    fps=cap.get(cv2.CAP_PROP_FPS)
                    if not np.isfinite(fps) or fps<=0: fps=args.fps
                    index=0
                    while True:
                        yield
                        if not key(): break
                        if paused: time.sleep(.02);continue
                        started=time.perf_counter()
                        ok,frame=cap.read()
                        if not ok: break
                        display(frame,index/fps);index+=1
                        if args.max_frames and pipeline.count>=args.max_frames: break
                        if not args.headless: time.sleep(max(0,1/fps-(time.perf_counter()-started)))
        except KeyboardInterrupt:
            pass
        finally:
            if latest is not None and args.output:
                dest=Path(args.output).expanduser();dest.parent.mkdir(parents=True,exist_ok=True)
                if not cv2.imwrite(str(dest),latest): raise RuntimeError(f'화면 저장 실패: {dest}')
                print(f'Saved {dest}',flush=True)
            if cap is not None: cap.release()
            if node is not None: node.destroy_node()
            if rclpy is not None and rclpy.ok(): rclpy.shutdown()
            if gui is not None: gui.shutdown()

    iterator=steps()
    if gui is None:
        for _ in iterator: pass
    else:
        gui.run(iterator)
