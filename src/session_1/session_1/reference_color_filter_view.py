"""참고자료 / 판단: RGB·HSV·YCrCb 색 범위를 슬라이더로 비교한다.

주 교육 파이프라인의 PIDNet을 대체하지 않는다. 각 필터는 독립 적용하며
세 결과를 AND/OR로 합치지 않는다. 인지/출력은 runtime.py에서 담당한다.
"""
import cv2
import numpy as np
from .drawing import panel

SPACES = {
    'RGB': (cv2.COLOR_BGR2RGB, ('R','G','B'), (255,255,255)),
    'HSV': (cv2.COLOR_BGR2HSV, ('H','S','V'), (179,255,255)),
    'YCrCb': (cv2.COLOR_BGR2YCrCb, ('Y','Cr','Cb'), (255,255,255)),
}
DEFAULTS = {'RGB': ([160,160,160],[255,255,255]),
            'HSV': ([0,0,160],[179,85,255]),
            'YCrCb': ([160,100,100],[255,155,155])}


def apply_filter(frame, space, lower, upper):
    """OpenCV는 입력이 BGR이다. RGB 화면은 변환 후 R/G/B 순서로 설정한다.

    하한 > 상한인 채널은 빈 마스크를 반환한다. 몰래 값을 뒤집지 않는다.
    HSV의 H는 0~179이며, 범위가 0을 가로지르는 색은 두 범위가 필요하다.
    """
    code,_,limits=SPACES[space]
    lower=np.asarray(lower,dtype=int);upper=np.asarray(upper,dtype=int)
    if lower.shape!=(3,) or upper.shape!=(3,) or np.any(lower<0) or np.any(upper<0) or np.any(lower>limits) or np.any(upper>limits):
        raise ValueError('색공간 채널 범위를 확인하세요')
    converted=cv2.cvtColor(frame,code)
    mask=cv2.inRange(converted,lower.astype(np.uint8),upper.astype(np.uint8))
    return mask,cv2.bitwise_and(frame,frame,mask=mask)


class ColorFilterView:
    def __init__(self):
        self.bounds={name:(list(lo),list(hi)) for name,(lo,hi) in DEFAULTS.items()}
        self.controls={};self.dirty=False

    def set_bound(self,space,channel,side,value):
        """Qt 슬라이더 값 변경을 영상 처리에 전달한다."""
        maximum=SPACES[space][2][channel]
        if not 0<=value<=maximum: raise ValueError('채널 범위 초과')
        self.bounds[space][side][channel]=int(value)
        self.dirty=True

    def render(self,frame):
        h,w=frame.shape[:2]
        panels=[panel(frame,'REFERENCE ONLY: original',
                      ['Independent color thresholds, no PIDNet',
                       'Adjust RGB / HSV / YCrCb tabs on the right',
                       'Slider changes also update a paused frame'])]
        metrics={}
        for space in SPACES:
            lo,hi=self.bounds[space]
            mask,filtered=apply_filter(frame,space,lo,hi)
            count=int(np.count_nonzero(mask));metrics[space]={'lower':lo.copy(),'upper':hi.copy(),'pixels':count}
            lines=[f'min={lo}  max={hi}',f'Selected pixels: {count} / {h*w}',
                   'EMPTY RANGE: min > max' if any(a>b for a,b in zip(lo,hi)) else 'Black = removed pixels']
            panels.append(panel(filtered,f'REFERENCE ONLY: {space}',lines))
        self.dirty=False
        return np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])]),metrics


def main(args=None):
    from .runtime import run
    run('reference_color_filter',args)

if __name__=='__main__': main()
