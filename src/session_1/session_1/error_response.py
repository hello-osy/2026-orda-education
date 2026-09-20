"""오차 수렴 교육용 모의 응답. 실제 차량 식별 모델/주행 예측이 아니다.

동일 초기 오차 20에서 가상의 관성계 x''=3*u-2*x'를 적분한다.
P와 PID는 같은 출력 제약을 사용한다. 결과가 0으로 수렴한다고 보장하지 않는다.
"""
from .pid_view import PID


def simulate_response(gains,duration=12.,dt=.02):
    controllers=(PID(gains[0],0.,0.),PID(*gains))
    curves=[]
    for controller in controllers:
        position=-20.;velocity=0.;points=[(0.,20.)]
        for step in range(1,round(duration/dt)+1):
            stamp=step*dt
            command=controller.update(0.,position,velocity,stamp).output
            velocity+=(3*command-2*velocity)*dt
            position+=velocity*dt
            points.append((stamp,-position))
        curves.append(points)
    return curves
