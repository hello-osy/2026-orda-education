"""Session 2의 센서 환산·PWM 상한을 사용한 목표각 정지 제어. 구동은 0."""
import math


def parse_feedback(line):
    fields = line.strip().split(',')
    if len(fields) != 4 or fields[0] != 'R':
        return None
    try:
        raw, volts, percent = int(fields[1]), float(fields[2]), int(fields[3])
    except ValueError:
        return None
    return raw if 0 <= raw <= 1023 and math.isfinite(volts) and 0 <= percent <= 100 else None


class SteeringControl:
    def __init__(self, enabled=False, center=-1, raw_span=80.0,
                 raw_direction=-1, pwm_direction=1):
        if not math.isfinite(raw_span) or raw_span <= 0:
            raise ValueError('raw_span은 양수여야 합니다.')
        if raw_direction not in (-1, 1) or pwm_direction not in (-1, 1):
            raise ValueError('방향은 -1 또는 1입니다.')
        if center != -1 and not 0 <= center <= 1023:
            raise ValueError('center_raw는 -1 또는 0~1023입니다.')
        self.enabled = enabled
        self.center = None if center == -1 else center
        self.raw_span = raw_span
        self.raw_direction = raw_direction
        self.pwm_direction = pwm_direction
        self.raw = self.angle = self.target = None
        self.last_requested = self.request_stamp = None
        self.feedback_stamp = self.target_stamp = self.heartbeat_stamp = None
        self.velocity = 0.0
        self.motion_sign = None
        self.motion_started = None
        self.pulse_until = self.next_feedback_after = None
        self.reached = self.expired = False
        self.reason = '중앙 보정 대기'

    def receive(self, raw, now):
        if not isinstance(raw, int) or not 0 <= raw <= 1023:
            return
        angle = None if self.center is None else self.raw_direction * (raw-self.center)*45/self.raw_span
        dt = 0.0 if self.feedback_stamp is None else now-self.feedback_stamp
        if angle is not None and self.angle is not None and 0 < dt <= .5:
            self.velocity = .25*(angle-self.angle)/dt + .75*self.velocity
        else:
            self.velocity = 0.0
        self.raw, self.angle, self.feedback_stamp = raw, angle, now

    def set_target(self, data, now):
        # 타입/범위 오류와 전진 요청은 이전 명령도 즉시 취소한다.
        self.request_stamp = now
        if (len(data) != 2 or any(not isinstance(v, int) for v in data)
                or not -30 <= data[0] <= 30 or data[1] != 0):
            self.stop('실차는 [조향각 -30~30, 0]만 허용합니다.')
            return False
        target = float(data[0])
        # 같은 목표를 10Hz로 반복해도 도달 상태와 출력 제한 시간을 초기화하지 않는다.
        if target == self.last_requested and self.target is None:
            return True  # 통신/피드백 오류 뒤 같은 명령 스트림으로 자동 재시작하지 않는다.
        if target != self.last_requested:
            self.motion_sign = self.motion_started = None
            self.reached = self.expired = False
        self.last_requested = target
        self.target, self.target_stamp = target, now
        return True

    def stop(self, reason='정지'):
        self.target = self.target_stamp = None
        self.reason = reason

    def calibrate(self, now):
        if self.request_stamp is not None and now-self.request_stamp <= .5:
            raise ValueError('명령 발행을 멈춘 뒤 중앙을 보정하세요.')
        if self.raw is None or self.feedback_stamp is None or now-self.feedback_stamp > .5:
            raise ValueError('최신 Arduino R 피드백이 없습니다.')
        self.register_center(self.raw)
        return self.center

    def register_center(self, raw, reason='중앙 보정 완료'):
        """부팅 안정화 완료 또는 명시적 수동 보정에서만 중앙값과 제어 상태를 갱신한다."""
        if not isinstance(raw, int) or isinstance(raw, bool) or not 0 <= raw <= 1023:
            raise ValueError('중앙 raw는 0~1023 정수여야 합니다.')
        self.center = raw
        self.angle = (None if self.raw is None else
                      self.raw_direction*(self.raw-self.center)*45/self.raw_span)
        self.velocity = 0.0
        self.last_requested = self.request_stamp = None
        self.motion_sign = self.motion_started = None
        self.pulse_until = self.next_feedback_after = None
        self.reached = self.expired = False
        self.stop(reason)

    def command(self, now):
        reason = ''
        if not self.enabled:
            reason = 'arm:=true 필요'
        elif self.center is None:
            reason = '중앙 보정 필요'
        elif self.heartbeat_stamp is None or now-self.heartbeat_stamp > .3:
            reason = 'ROS 처리 중단'
        elif self.feedback_stamp is None or now-self.feedback_stamp > .5:
            reason = '조향 피드백 끊김'
        elif self.target is None or self.target_stamp is None or now-self.target_stamp > .5:
            reason = '명령 수신 끊김'
        elif self.angle is None or not math.isfinite(self.angle) or abs(self.angle) > 45:
            reason = '조향 범위 초과: 중앙/방향 설정 확인'
        if reason:
            self.stop(reason)
            return b'0 0\n'
        error = self.target-self.angle
        if self.motion_sign is None:
            self.motion_sign = 1 if error > 0 else -1
            self.motion_started = now
        # 허용 오차 1도에 들어오거나 목표를 지나치면 즉시 끈다. 자동 역회전하지 않는다.
        if abs(error) <= 1 or self.motion_sign*error <= 0:
            self.reached = True
        if not self.reached and now-self.motion_started >= 5.0:
            self.expired = True
        pwm = 0
        if not self.reached and not self.expired:
            # 실차 R 피드백은 ROS 발행 주기보다 느릴 수 있다. 같은 센서값으로
            # 계속 밀지 않고 150을 20ms 적용한 뒤 멈춰 새 측정과 관성 정착을 기다린다.
            if self.pulse_until is None or (now >= self.pulse_until
                    and self.feedback_stamp > self.next_feedback_after):
                self.pulse_until = now+.02
                self.next_feedback_after = self.pulse_until+.15
            if now < self.pulse_until:
                pwm = 150*self.motion_sign*self.pwm_direction
        state = '목표 도달: 출력 0' if self.reached else (
            '5초 내 목표 미도달: 출력 0' if self.expired else f'조향 PWM {pwm:+}')
        self.reason = f'목표 {self.target:+.0f}도 / 센서 환산 {self.angle:+.1f}도 / {state} / 구동 0'
        return f'{pwm} 0\n'.encode('ascii')
