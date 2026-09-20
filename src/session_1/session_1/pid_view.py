"""제어 / PID: 목표 조향각과 실제 조향각을 비교하여 모터 PWM을 계산한다.

원본 drive_control_node.py의 calculate_steer_pwm, raw_to_steer_angle,
steering_feedback_callback에서 계산 부분만 이식했다. ROS/아두이노 출력은 제외.
판단: PIDNet → scan line → 픽셀 오차를 목표 조향각으로 변환.
제어: 조향각 오차 → PID → PWM. 픽셀 오차를 PID에 직접 넣지 않는다.
"""
from collections import deque
from dataclasses import dataclass
import math
import cv2
import numpy as np

@dataclass
class PIDResult:
    target: float | None = None
    angle: float | None = None
    error: float | None = None
    p: float = 0.
    i: float = 0.
    d: float = 0.
    output: int = 0
    raw: float = 0.
    dt: float = 0.
    status: str = 'RESET'

class PID:
    """원본 시간주행 PID: P=6.5, I=0, D=0.8, PWM 상한150/최소40."""
    def __init__(self,kp=6.5,ki=0.,kd=.8):
        if not all(math.isfinite(v) and v>=0 for v in (kp,ki,kd)):
            raise ValueError('PID gain은 유한한 0 이상 값이어야 합니다')
        self.kp,self.ki,self.kd=kp,ki,kd
        self.reset()

    def reset(self):
        self.integral=0.;self.stamp=None;self.target=0.

    def update(self,target,angle,velocity,stamp):
        if target is None or angle is None:
            self.reset();return PIDResult(target=target,angle=angle,status='MISSING LANE / FEEDBACK')
        if not all(math.isfinite(x) for x in (target,angle,velocity,stamp)):
            self.reset();return PIDResult(status='INVALID INPUT')
        gap=0.05 if self.stamp is None else stamp-self.stamp
        if gap<=0 or gap>1.: self.reset();gap=.05
        dt=max(.001,min(gap,.1));self.stamp=stamp
        target=float(np.clip(target,-45,45))
        # 원본: 새로운 목표로 오차 부호가 바뀌면 적분 초기화.
        if (self.target-angle)*(target-angle)<0: self.integral=0.
        self.target=target;error=target-angle
        if abs(error)<=1.:
            self.integral=0.
            return PIDResult(target=target,angle=angle,error=error,dt=dt,status='WITHIN 1 DEG')
        candidate=self.integral+error*dt
        candidate=float(np.clip(candidate,-30/self.ki,30/self.ki)) if self.ki>0 else 0.
        p=self.kp*error
        # 원본과 같은 derivative on measurement: 목표 급변에 의한 D kick 억제.
        d=-self.kd*velocity
        candidate_output=p+self.ki*candidate+d
        if not (abs(candidate_output)>150 and candidate_output*error>0):
            self.integral=candidate
        i=self.ki*self.integral;raw=p+i+d
        output=float(np.clip(raw,-150,150))
        if 0<abs(output)<40: output=40. if output>0 else -40.
        return PIDResult(target,angle,error,p,i,d,int(round(output)),raw,dt,
                         'SATURATED' if abs(raw)>150 else 'TRACKING')

class Feedback:
    """A1(raw)→실제 각도. 첫 유효 raw를 중앙값으로 캡처하는 원본 규약."""
    def __init__(self):
        self.center=None;self.angle=None;self.velocity=0.;self.stamp=None;self.received=None

    def update(self,raw,stamp,received):
        if not 0<=raw<=1023: return
        if self.stamp is not None and stamp<self.stamp:
            self.center=None;self.angle=None;self.velocity=0.;self.stamp=None
        if self.center is None: self.center=raw
        angle=float(np.clip((self.center-raw)*45/80,-45,45))
        dt=0. if self.stamp is None else stamp-self.stamp
        if self.angle is not None and 0<dt<=1.:
            self.velocity=.25*(angle-self.angle)/dt+.75*self.velocity
        else: self.velocity=0.
        self.angle=angle;self.stamp=stamp;self.received=received

class History:
    def __init__(self): self.values=deque(maxlen=180)
    def add(self,error,result): self.values.append((result.target,result.angle,result.output))

class DemoPlant:
    """교육용 모의 조향 축. 실제 차량/주행 성능을 재현한 모델이 아니다."""
    def __init__(self): self.reset()
    def reset(self): self.angle=0.;self.velocity=0.;self.pwm=0.;self.stamp=None
    def step(self,stamp):
        dt=0 if self.stamp is None else stamp-self.stamp
        if dt<=0 or dt>1: self.reset();dt=0
        previous=self.angle
        self.angle=float(np.clip(self.angle+self.pwm*.3*dt,-45,45))
        raw_velocity=(self.angle-previous)/dt if dt else 0.
        self.velocity=.25*raw_velocity+.75*self.velocity
        self.stamp=stamp
        return self.angle,self.velocity

def visualize(frame,labels,scan,result,history,feedback_mode):
    from .drawing import panel,stack,trace,text
    before=frame.copy()
    cv2.line(before,(round(scan.target_x),0),(round(scan.target_x),frame.shape[0]-1),(0,150,255),2)
    if scan.measured_x is not None:
        cv2.circle(before,(round(scan.measured_x),scan.y),8,(0,255,255),-1)
    def angle_text(value): return 'unavailable' if value is None else f'{value:+.1f}deg'
    before=panel(before,'BEFORE: target / actual angle',
                 [f'target={angle_text(result.target)} actual={angle_text(result.angle)}',
                  f'angle error={angle_text(result.error)}  ({feedback_mode})',
                  'Input: PIDNet -> scan line -> target angle'])
    after=np.full_like(frame,24)
    h,w=after.shape[:2];center=(w//2,h//2);radius=min(w,h)//3
    cv2.circle(after,center,radius,(100,100,100),3)
    for value,color in [(result.target,(0,150,255)),(result.angle,(255,220,0))]:
        if value is None: continue  # 미검출/피드백 없음은 직진 0도로 그리지 않는다.
        a=math.radians(value-90)
        tip=(round(center[0]+radius*math.cos(a)),round(center[1]+radius*math.sin(a)))
        cv2.arrowedLine(after,center,tip,color,4,tipLength=.2)
    text(after,'orange: target / cyan: actual',(20,h-30))
    after=panel(after,'AFTER: steering motor PWM',
                [f'P={result.p:+.1f} I={result.i:+.1f} D={result.d:+.1f}',
                 f'raw={result.raw:+.1f} PWM={result.output:+d} dt={result.dt:.3f}s',result.status])
    top=stack(before,after)
    left=trace([x[0] for x in history.values],w,130,'Target angle [deg]',45)
    right=trace([x[2] for x in history.values],w,130,'Motor PWM',150)
    return np.vstack([top,stack(left,right)])

def main(args=None):
    from .runtime import run
    run('pid', args)

if __name__ == '__main__': main()
