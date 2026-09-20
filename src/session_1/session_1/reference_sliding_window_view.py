"""참고자료 / 판단: 차선 이진 마스크를 아래에서 위로 sliding window로 추적.

PIDNet의 실선/점선 마스크를 입력으로 사용한다. 주 교육의 scan line 대신
사용할 수 있는 기법 설명용이며, 주 파이프라인의 조향 계산에는 연결하지 않는다.
"""
from dataclasses import dataclass
import cv2
import numpy as np
from .drawing import panel,stack

@dataclass
class Tracking:
    mask: np.ndarray
    windows: list
    centers: list
    curve: np.ndarray | None
    base_x: int | None


def track_lane(labels,lane='right',count=9,margin=50,min_pixels=20):
    """하단 histogram으로 시작하고, 픽셀 평균 x로 다음 창을 이동한다.

    빈 창에서는 검색 중심만 유지하고 검출점은 추가하지 않는다. 세 개 이상의
    창에서 검출되었을 때만 x=f(y) 2차식으로 피팅하며 검출 y 구간만 표시한다.
    """
    if count<3 or margin<1 or min_pixels<1: raise ValueError('windows>=3, margin>=1, min_pixels>=1')
    if lane not in ('right','center'): raise ValueError('lane은 right/center')
    h,w=labels.shape
    if count>h: raise ValueError('window 개수는 영상 높이 이하여야 합니다')
    mask=((labels==(2 if lane=='right' else 3))*255).astype(np.uint8)
    histogram=np.count_nonzero(mask[h*2//3:],axis=0)
    if lane=='right': histogram[:w//2]=0
    if not histogram.any(): return Tracking(mask,[],[],None,None)
    # 평평한 histogram 최고점 구간의 중앙에서 출발한다.
    peak=int(np.argmax(histogram));left=peak;right=peak
    while left>0 and histogram[left-1]==histogram[peak]: left-=1
    while right<w-1 and histogram[right+1]==histogram[peak]: right+=1
    current=(left+right)//2;base=current
    ys,xs=np.nonzero(mask);windows=[];centers=[]
    edges=np.linspace(h,0,count+1,dtype=int)
    for i in range(count):
        y0,y1=int(edges[i+1]),int(edges[i]);x0=max(0,current-margin);x1=min(w,current+margin+1)
        keep=(ys>=y0)&(ys<y1)&(xs>=x0)&(xs<x1)
        found=np.count_nonzero(keep)>=min_pixels
        windows.append((x0,y0,x1,y1,bool(found)))
        if found:
            current=int(round(float(xs[keep].mean())))
            centers.append((float(xs[keep].mean()),float(ys[keep].mean())))
    curve=None
    if len(centers)>=3:
        points=np.asarray(centers)
        coeff=np.polyfit(points[:,1],points[:,0],2)
        y=np.arange(int(np.ceil(points[:,1].min())),int(np.floor(points[:,1].max()))+1)
        x=np.polyval(coeff,y)
        valid=(x>=0)&(x<w)
        curve=np.column_stack([x[valid],y[valid]]).astype(np.int32)
    return Tracking(mask,windows,centers,curve,base)


def visualize(frame,labels,lane='right',count=9,margin=50,min_pixels=20):
    result=track_lane(labels,lane,count,margin,min_pixels)
    before=cv2.cvtColor(result.mask,cv2.COLOR_GRAY2BGR);after=frame.copy()
    for i,(x0,y0,x1,y1,found) in enumerate(result.windows):
        color=(0,220,0) if found else (0,160,255)
        cv2.rectangle(after,(x0,y0),(x1-1,y1-1),color,2)
        cv2.putText(after,str(i+1),(x0+3,min(y1-2,y0+16)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
    for x,y in result.centers:cv2.circle(after,(round(x),round(y)),4,(255,255,0),-1)
    if result.curve is not None and len(result.curve)>1:cv2.polylines(after,[result.curve],False,(255,0,255),3)
    status='TRACKING' if result.curve is not None else 'INSUFFICIENT POINTS / LANE LOST'
    canvas=stack(panel(before,'REFERENCE ONLY: lane mask',[f'PIDNet class: {lane}', 'Histogram seed from bottom 1/3']),
                 panel(after,'REFERENCE ONLY: sliding windows',
                       [f'Bottom -> top / {len(result.centers)} valid windows',
                        'Green: found / orange: empty / magenta: fit',status]))
    return canvas,{'base_x':result.base_x,'valid_windows':len(result.centers),'status':status}


def main(args=None):
    from .runtime import run
    run('reference_sliding_window',args)

if __name__=='__main__': main()
