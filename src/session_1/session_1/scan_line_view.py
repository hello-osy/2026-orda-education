"""판단 2 / scan line: 차선 마스크의 가로 띠에서 현재 x를 측정한다.

PPT의 기준값 - 현재값을 error_px로 정의한다. 영상 x는 오른쪽이 +이다.
오른쪽 차선 기준 error가 +이면 차가 오른쪽으로 치우친 상태이므로,
PID 단계에서 steering_sign=-1을 곱해 좌조향(음수)으로 바꾼다.
원본의 measured-target 조향 부호와 결과적으로 같다.
"""
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass
class ScanResult:
    y: int
    target_x: float
    measured_x: float | None
    error_px: float | None
    mask: np.ndarray
    candidates: tuple

def scan_line(labels, y_ratio=.75, target_ratio=.79, lane='right', band=5):
    """실선/점선을 섞지 않고 연속 픽셀 덩어리별 중심을 구한다.

    right: 오른쪽 절반의 실선 중 가장 오른쪽 덩어리.
    center: 중앙 점선 중 목표 x에 가장 가까운 덩어리.
    미검출은 None이며, 정상 오차 0과 구분한다. 자동 기준 전환은 하지 않는다.
    """
    h,w = labels.shape
    if not 0 <= y_ratio <= 1 or not 0 <= target_ratio <= 1 or band < 1:
        raise ValueError('scan_y/target_x는 0~1, band는 1 이상이어야 합니다')
    if lane not in ('right','center'): raise ValueError('lane: right 또는 center')
    y = round(y_ratio*(h-1)); target = target_ratio*(w-1)
    mask = ((labels == (2 if lane == 'right' else 3))*255).astype(np.uint8)
    rows = mask[max(0,y-band//2):min(h,y+band//2+1)]
    xs = np.flatnonzero(np.count_nonzero(rows,axis=0) >= max(1,len(rows)//2+1))
    groups = np.split(xs,np.flatnonzero(np.diff(xs)>1)+1)
    candidates = tuple(float(g.mean()) for g in groups if len(g)>=2)
    eligible = [x for x in candidates if lane != 'right' or x >= w/2]
    measured = (max(eligible) if lane == 'right' else min(eligible,key=lambda x:abs(x-target))) if eligible else None
    return ScanResult(y,target,measured,None if measured is None else target-measured,mask,candidates)

def visualize(frame, labels, result):
    from .drawing import panel, stack
    before = cv2.cvtColor(result.mask,cv2.COLOR_GRAY2BGR)
    after = cv2.addWeighted(frame,.5,before,.5,0)
    h,w = labels.shape
    cv2.line(after,(0,result.y),(w-1,result.y),(255,220,0),2)
    tx = round(result.target_x)
    cv2.line(after,(tx,0),(tx,h-1),(0,150,255),2)
    for x in result.candidates: cv2.circle(after,(round(x),result.y),4,(160,160,160),-1)
    if result.measured_x is not None:
        x = round(result.measured_x)
        cv2.circle(after,(x,result.y),7,(0,255,255),-1)
        cv2.arrowedLine(after,(x,result.y),(tx,result.y),(255,255,0),2)
        detail = [f'target={result.target_x:.1f}px  measured={result.measured_x:.1f}px',
                  f'error = target - measured = {result.error_px:+.1f}px']
    else: detail = ['LANE LOST: no measurement', 'error=None (not zero)']
    return stack(panel(before,'BEFORE: lane mask',['PIDNet output: selected lane class']),
                 panel(after,'AFTER: scan line',detail))

def main(args=None):
    from .runtime import run
    run('scan_line', args)

if __name__ == '__main__': main()
