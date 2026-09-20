"""OpenCV 교육 화면 공통 요소. 폰트 의존성 없이 영문 라벨, 설명은 한국어 문서."""
import cv2
import numpy as np

def text(image,s,xy,color=(235,235,235),scale=.48):
    cv2.putText(image,s,xy,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)

def panel(image,title,lines):
    h,w=image.shape[:2]
    canvas=np.full((h+112,w,3),24,np.uint8)
    canvas[36:36+h]=image
    text(canvas,title,(12,25),(0,220,255),.6)
    for i,line in enumerate(lines): text(canvas,line,(10,h+56+i*19))
    return canvas

def stack(a,b): return np.hstack([a,b])

def trace(values,width,height,title,limit):
    out=np.full((height,width,3),24,np.uint8)
    text(out,f'{title}  range +/-{limit:g}  last 180 samples',(10,18))
    mid=(height+24)//2
    cv2.line(out,(0,mid),(width-1,mid),(75,75,75),1)
    previous=None
    for i,value in enumerate(values):
        if value is None: previous=None; continue
        point=(int(i*(width-1)/179),int(mid-np.clip(value/limit,-1,1)*(height-30)/2))
        if previous: cv2.line(out,previous,point,(0,220,255),2)
        previous=point
    return out
