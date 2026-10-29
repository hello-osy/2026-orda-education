"""펌웨어의 중앙 맞춤이 끝난 뒤 안정된 센서값만 등록한다. 모터를 제어하지 않는다."""
from collections import deque
import math
import statistics


class StartupCenter:
    """부팅 대기 후 2초 동안 연속 수신한 실제 R 프레임의 중앙값을 한 번 정한다."""
    def __init__(self, opened_at, startup_wait=10.0, stable_window=2.0,
                 min_frames=8, raw_tolerance=3, max_gap=.5):
        if not math.isfinite(opened_at):
            raise ValueError('opened_at은 유한한 시각이어야 합니다.')
        if not math.isfinite(startup_wait) or startup_wait < 0:
            raise ValueError('startup_wait는 0 이상의 초 단위 값이어야 합니다.')
        if not math.isfinite(stable_window) or stable_window <= 0:
            raise ValueError('stable_window는 양수여야 합니다.')
        if not isinstance(min_frames, int) or min_frames < 2:
            raise ValueError('min_frames는 2 이상의 정수여야 합니다.')
        if not isinstance(raw_tolerance, int) or raw_tolerance < 0:
            raise ValueError('raw_tolerance는 0 이상의 정수여야 합니다.')
        if not math.isfinite(max_gap) or max_gap <= 0:
            raise ValueError('max_gap은 양수여야 합니다.')
        self.ready_after = opened_at + startup_wait
        self.startup_wait = startup_wait
        self.stable_window = stable_window
        self.min_frames = min_frames
        self.raw_tolerance = raw_tolerance
        self.max_gap = max_gap
        self.samples = deque()
        self.center = None

    def status(self, now):
        if self.center is not None:
            return f'자동 중앙 등록 완료: raw={self.center}'
        if now < self.ready_after:
            return f'Arduino 부팅 대기 ({self.startup_wait:g}초), 조향·구동 출력 0'
        return ('중앙 raw 안정화 대기 '
                f'({self.stable_window:g}초 / {self.min_frames}회 이상 / 변화폭 {self.raw_tolerance}), 출력 0')

    def receive(self, raw, now):
        """새 실제 시리얼 프레임마다 호출한다. 등록이 완료된 그 한 번만 int를 반환한다."""
        if self.center is not None:
            return None
        if (not isinstance(raw, int) or isinstance(raw, bool) or not 0 <= raw <= 1023
                or not math.isfinite(now)):
            self.samples.clear()
            return None
        if now < self.ready_after:
            self.samples.clear()
            return None
        if self.samples and (now < self.samples[-1][0]
                             or now-self.samples[-1][0] > self.max_gap + 1e-9):
            self.samples.clear()
        self.samples.append((now, raw))
        # 현재 2초 구간을 덮는 첫 표본 하나를 남긴다. 정확한 시각 일치를 요구하지 않는다.
        while len(self.samples) > 1 and self.samples[1][0] <= now-self.stable_window:
            self.samples.popleft()
        # 움직이거나 튄 센서값이 포함되면 연속 안정 구간을 다시 확보한다.
        while self.samples and (max(value for _, value in self.samples)
                                - min(value for _, value in self.samples) > self.raw_tolerance):
            self.samples.popleft()
        if (len(self.samples) < self.min_frames
                or now-self.samples[0][0] + 1e-9 < self.stable_window):
            return None
        self.center = round(statistics.median(value for _, value in self.samples))
        self.samples.clear()
        return self.center
