"""주 교육용: segmentation / scan line / PID의 AFTER만 한 화면에 표시.

같은 프레임에 모델은 한 번만 실행한다. 왼쪽부터 판단(픽셀 분류),
판단(주행 오차), 제어(PID PWM)이다. BEFORE 원본 비교 칸은 표시하지 않는다.
"""
import numpy as np
from . import segmentation_view,scan_line_view,pid_view
from .drawing import text


def visualize(frame,labels,scan,result,history,feedback_mode):
    # 기존 수업 화면의 AFTER 패널을 재사용해 개별 실행과 설명이 일치하게 한다.
    w=frame.shape[1];panel_height=frame.shape[0]+112
    segmentation=segmentation_view.visualize(frame,labels)[:,w:]
    scan_panel=scan_line_view.visualize(frame,labels,scan)[:,w:]
    pid_panel=pid_view.visualize(frame,labels,scan,result,history,feedback_mode)[:panel_height,w:]
    # PID의 BEFORE 칸을 생략해도 피드백 출처가 사라지지 않게 명시한다.
    text(pid_panel,feedback_mode,(12,frame.shape[0]-5),(0,220,255),.55)
    return np.hstack([segmentation,scan_panel,pid_panel])


def main(args=None):
    from .runtime import run
    run('pipeline_after',args)

if __name__=='__main__': main()
