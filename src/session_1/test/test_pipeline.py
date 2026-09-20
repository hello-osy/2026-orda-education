from types import SimpleNamespace
import numpy as np
import pytest
from session_1.scan_line_view import scan_line
from session_1.pid_view import PID,Feedback
from session_1.runtime import decode_image


def test_separate_lanes_and_sign():
    labels=np.zeros((100,640),np.uint8)
    labels[:,100:110]=2;labels[:,300:310]=3;labels[:,500:510]=2
    right=scan_line(labels,target_ratio=.79)
    assert right.measured_x==504.5
    assert right.error_px==pytest.approx(.79*639-504.5)
    assert scan_line(labels,lane='center').measured_x==304.5
    assert scan_line(np.zeros_like(labels)).error_px is None


def test_padding_and_rgb():
    msg=SimpleNamespace(encoding='rgb8',height=1,width=2,step=8,
                        data=bytes([255,0,0,0,255,0,99,99]))
    assert decode_image(msg).tolist()==[[[0,0,255],[0,255,0]]]
    msg.encoding='invalid'
    with pytest.raises(ValueError):decode_image(msg)


def test_yuy2_and_malformed():
    msg=SimpleNamespace(encoding='yuv422_yuy2',height=1,width=2,step=4,data=bytes([128,128,128,128]))
    assert decode_image(msg).shape==(1,2,3)
    msg.step=3
    with pytest.raises(ValueError):decode_image(msg)


def test_original_pid_terms_and_limits():
    p=PID()
    r=p.update(10,0,2,1.)
    assert r.p==65 and r.i==0 and r.d==-1.6 and r.output==63
    assert p.update(45,0,0,1.1).output==150
    assert p.update(-45,0,0,1.2).output==-150
    assert p.update(2,0,0,1.3).output==40
    assert p.update(.5,0,0,1.4).output==0
    assert p.update(None,0,0,1.5).output==0


def test_derivative_on_measurement_no_target_kick():
    p=PID();p.update(0,0,0,1.)
    assert p.update(20,0,0,1.05).d==0


def test_integral_antiwindup_and_rewind():
    p=PID(kp=6.5,ki=1,kd=0)
    for t in np.arange(1,2,.05):p.update(45,0,0,float(t))
    assert p.integral==0
    p.update(2,0,0,3.);assert p.integral>0
    r=p.update(2,0,0,1.)
    assert r.dt==.05 and p.integral==pytest.approx(.1)


def test_feedback_calibration_filter_and_bad_raw():
    f=Feedback();f.update(480,1.,0.)
    assert f.angle==0
    f.update(400,1.1,.1);assert f.angle==45
    assert f.velocity==pytest.approx(112.5)
    f.update(2000,1.2,.2);assert f.angle==45
    f.update(560,1.3,.3);assert f.angle==-45
    f.update(500,.5,.4);assert f.angle==0


def test_reference_color_spaces_and_inverted_range():
    from session_1.reference_color_filter_view import apply_filter
    frame=np.array([[[0,0,255],[255,0,0],[255,255,255]]],np.uint8)
    rgb,_=apply_filter(frame,'RGB',[250,0,0],[255,10,10])
    assert rgb.tolist()==[[255,0,0]]  # BGR를 R/G/B로 올바르게 해석
    hsv,_=apply_filter(frame,'HSV',[0,240,240],[5,255,255])
    assert hsv.tolist()==[[255,0,0]]
    ycc,_=apply_filter(frame,'YCrCb',[250,120,120],[255,135,135])
    assert ycc.tolist()==[[0,0,255]]
    empty,_=apply_filter(frame,'RGB',[200,0,0],[100,255,255])
    assert not empty.any()


def test_reference_slider_callbacks():
    from session_1.reference_color_filter_view import ColorFilterView,SPACES
    view=ColorFilterView()
    view.set_bound('RGB',0,0,255)
    assert view.dirty
    frame=np.full((64,320,3),200,np.uint8)
    _,metrics=view.render(frame)
    assert metrics['RGB']['pixels']==0 and not view.dirty
    assert metrics['HSV']['pixels']==320*64
    for space,(_,channels,limits) in SPACES.items():
        for i,maximum in enumerate(limits):
            for side in (0,1):
                view.set_bound(space,i,side,maximum//2)
                assert view.bounds[space][side][i]==maximum//2


def test_sliding_windows_follow_curve_and_mark_loss():
    from session_1.reference_sliding_window_view import track_lane
    labels=np.zeros((180,320),np.uint8)
    for y in range(180):
        x=int(200+.001*(y-90)**2)
        labels[y,x-3:x+4]=2
    tracked=track_lane(labels,count=9,margin=20,min_pixels=10)
    assert len(tracked.centers)==9 and tracked.curve is not None
    expected=200+.001*(tracked.curve[:,1]-90)**2
    assert np.max(np.abs(tracked.curve[:,0]-expected))<3
    labels[60:100]=0
    lost=track_lane(labels,count=9,margin=20,min_pixels=10)
    assert any(not window[-1] for window in lost.windows)
    assert track_lane(np.zeros_like(labels)).curve is None


def test_after_only_layout():
    from session_1 import pipeline_after_view
    from session_1.pid_view import PIDResult,History
    frame=np.zeros((128,320,3),np.uint8);labels=np.zeros((128,320),np.uint8)
    scan=scan_line(labels)
    canvas=pipeline_after_view.visualize(frame,labels,scan,PIDResult(),History(),'RECORDED A1')
    assert canvas.shape==(128+112,960,3)


def test_missing_feedback_not_shown_as_zero_angle():
    from session_1 import pid_view,pipeline_after_view
    result=PID().update(20,None,0,1.)
    assert result.target==20 and result.angle is None and result.error is None
    assert result.output==0
    frame=np.zeros((128,320,3),np.uint8);labels=np.zeros((128,320),np.uint8)
    scan=scan_line(labels);history=pid_view.History();history.add(None,result)
    assert history.values[-1][1] is None
    assert pipeline_after_view.visualize(frame,labels,scan,result,history,'RECORDED A1').shape==(240,960,3)


def test_invalid_packed_image_size():
    msg=SimpleNamespace(encoding='yuv422_yuy2',height=1,width=3,step=6,data=bytes(6))
    with pytest.raises(ValueError,match='짝수'):decode_image(msg)
    msg.width=0
    with pytest.raises(ValueError,match='양수'):decode_image(msg)


def test_compressed_image_decode():
    import cv2
    frame=np.full((20,40,3),123,np.uint8)
    ok,data=cv2.imencode('.png',frame);assert ok
    assert np.array_equal(decode_image(SimpleNamespace(data=data.tobytes()),True),frame)
    with pytest.raises(ValueError):decode_image(SimpleNamespace(data=b'not an image'),True)


def test_frozen_color_frame_redraw_on_slider_change(monkeypatch,tmp_path):
    from session_1 import runtime
    from session_1.reference_color_filter_view import ColorFilterView
    import cv2
    path=tmp_path/'input.png';cv2.imwrite(str(path),np.full((64,320,3),200,np.uint8))
    view=ColorFilterView();renders=[]
    real_render=view.render
    def render(frame):
        image,metrics=real_render(frame);renders.append(metrics['RGB']['pixels']);return image,metrics
    view.render=render
    monkeypatch.setattr(runtime.reference_color_filter_view,'ColorFilterView',lambda:view)
    keys=iter([32,-1,ord('q')])
    def key():
        k=next(keys)
        if k==-1:view.set_bound('RGB',0,0,255)
        return k
    gui=FakeGui(key);monkeypatch.setattr(runtime,'make_gui',lambda *a:gui)
    runtime.run('reference_color_filter',['--source','image','--input',str(path),'--width','320'])
    assert renders==[64*320,0] and gui.closed


@pytest.mark.parametrize('exit_key,visible',[(ord('q'),1),(27,1),(-1,0)])
def test_gui_exit_paths_cleanup(monkeypatch,tmp_path,exit_key,visible):
    from session_1 import runtime
    import cv2
    path=tmp_path/'input.png';cv2.imwrite(str(path),np.zeros((64,320,3),np.uint8))
    gui=FakeGui(lambda:exit_key if visible else ord('q'))
    monkeypatch.setattr(runtime,'make_gui',lambda *a:gui)
    out=tmp_path/'result.png'
    runtime.run('reference_color_filter',['--source','image','--input',str(path),'--width','320','--output',str(out)])
    assert gui.closed and cv2.imread(str(out)) is not None


def test_video_pause_resume_and_reset(monkeypatch):
    from session_1 import runtime
    import cv2
    seen={};real_pipeline=runtime.Pipeline
    def factory(stage,args):
        pipe=real_pipeline(stage,args);pipe.pid.integral=10.;pipe.history.values.append((1.,1.,1.));seen['pipe']=pipe;return pipe
    monkeypatch.setattr(runtime,'Pipeline',factory)
    class Capture:
        count=0;released=False
        def isOpened(self):return True
        def get(self,*a):return 30.
        def read(self):self.count+=1;return True,np.zeros((64,320,3),np.uint8)
        def release(self):self.released=True
    cap=Capture();monkeypatch.setattr(cv2,'VideoCapture',lambda *a:cap)
    keys=iter([32,-1,ord('r'),32,-1,ord('q')])
    gui=FakeGui(lambda:next(keys));monkeypatch.setattr(runtime,'make_gui',lambda *a:gui)
    runtime.run('reference_color_filter',['--source','video','--input','stub','--width','320'])
    assert cap.count==2 and cap.released
    assert seen['pipe'].pid.integral==0 and not seen['pipe'].history.values
    assert gui.pauses==[True,False] and gui.resets==1


class FakeGui:
    """런타임 상태 전이 테스트 대역. 실제 위젯 조작은 별도 UI 검증."""
    def __init__(self,poll): self.poll=poll;self.closed=False;self.pauses=[];self.resets=0
    def show_frame(self,*a):pass
    def set_paused(self,value):self.pauses.append(value)
    def mark_reset(self):self.resets+=1
    def run(self,steps):
        for _ in steps:pass
    def shutdown(self):self.closed=True
