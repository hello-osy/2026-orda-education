"""초보자용 통합 수업 창. UI는 메인 스레드, bag 읽기/모델 추론은 작업 스레드.

인지(녹화 센서) → 판단(segmentation/scan line) → 제어(PID)의 위치를
항상 같은 순서로 보여준다. 전후 비교는 반드시 같은 카메라 프레임이다.
"""
import argparse
import bisect
import sys
import time
from pathlib import Path
import cv2
import numpy as np
from PySide6.QtCore import Qt,QObject,QThread,QTimer,Signal,Slot
from PySide6.QtGui import QImage,QPixmap,QShortcut,QKeySequence
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QLabel,QVBoxLayout,QHBoxLayout,
    QPushButton,QSlider,QComboBox,QDoubleSpinBox,QGroupBox,QFormLayout,QSizePolicy,QButtonGroup)
from .lesson_player import LessonEngine,find_bag
from .pid_comparison import PIDComparison
from .error_plot import ErrorView
from collections import deque
from .segmentation_view import PALETTE
from .reference_color_filter_view import SPACES,DEFAULTS,apply_filter
from .reference_sliding_window_view import track_lane

LESSONS=[
 ('전체 흐름','카메라를 보고 → 주행 오차를 구하고 → 오차가 0에 수렴하도록 조향값과 속도값을 조절해요.','세 칸은 같은 순간의 결과예요. 이 화면에서는 제어 중 조향 제어를 살펴봐요.'),
 ('1. 이미지 전처리 & 차선 검출','Segmentation · 사진 속 도로와 차선을 색으로 구분해요.','왼쪽은 처리 전, 오른쪽은 처리 후예요. 일시정지를 눌러 같은 순간을 살펴보세요.'),
 ('2. 주행 오차 계산','Scan line · 가로선 위에서 차선이 어디에 있는지 찾아요.','주황 선은 원하는 위치, 노란 점은 찾은 차선이에요. 두 위치의 차이를 비교하세요.'),
 ('3. 조향 안정화','PID · 조향값을 조절해 오차가 0에 수렴하도록 해요.','왼쪽은 오차의 변화, 오른쪽은 녹화 입력으로 계산한 P/PID 출력이에요.'),
 ('참고: 색 필터','색 범위에 맞는 부분만 남겨보세요. AI 모델 없이 색만 사용해요.','오른쪽 슬라이더를 움직이면 결과가 바뀌어요. 멈춘 상태에서도 조절할 수 있어요.'),
 ('참고: 차선 추적','Sliding window · 작은 창을 아래에서 위로 옮기며 차선을 따라가요.','초록 창은 차선을 찾은 곳, 주황 창은 못 찾은 곳, 분홍 선은 연결한 결과예요.')]


class Worker(QObject):
    ready=Signal(object);frame=Signal(object);failed=Signal(str)
    def __init__(self,bag,device):
        super().__init__();self.bag=bag;self.device=device;self.engine=None
    @Slot()
    def load(self):
        try:
            self.engine=LessonEngine(self.bag,self.device)
            self.ready.emit(self.engine.bag.times)
        except Exception as exc: self.failed.emit(str(exc))
    @Slot(int,object,bool)
    def process(self,index,settings,reset):
        try: self.frame.emit(self.engine.process(index,settings,reset))
        except Exception as exc: self.failed.emit(str(exc))
    @Slot()
    def stop(self):
        if self.engine:self.engine.close()
        QThread.currentThread().quit()


class Picture(QGroupBox):
    def __init__(self):
        super().__init__();box=QVBoxLayout(self)
        self.image=QLabel('영상을 준비하고 있어요');self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(160,140);self.image.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Expanding)
        self.image.setStyleSheet('background:#101c30;border-radius:12px;color:#dbeafe;')
        self.caption=QLabel();self.caption.setWordWrap(True);self.caption.setMinimumHeight(65)
        box.addWidget(self.image,1);box.addWidget(self.caption);self.pixmap=None
    def display(self,title,frame,caption):
        self.setTitle(title);self.caption.setText(caption)
        rgb=np.ascontiguousarray(frame[:,:,::-1]);h,w=rgb.shape[:2]
        self.pixmap=QPixmap.fromImage(QImage(rgb.data,w,h,rgb.strides[0],QImage.Format_RGB888).copy());self.fit()
    def fit(self):
        if self.pixmap:self.image.setPixmap(self.pixmap.scaled(self.image.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
    def resizeEvent(self,event):super().resizeEvent(event);self.fit()


class Classroom(QMainWindow):
    request=Signal(int,object,bool);stop_worker=Signal()
    def __init__(self,bag,device='auto'):
        super().__init__();self.setWindowTitle('ORDA · 처음 만나는 자율주행')
        self.times=[];self.data=None;self.busy=False;self.playing=False;self.current=0
        self.position=0.;self.anchor=time.monotonic();self.stage=0;self.pending_reset=False
        self.closing=False;self.loaded=False;self.requested_index=0;self.generation=0
        self.bounds={s:(list(lo),list(hi)) for s,(lo,hi) in DEFAULTS.items()}
        self.error_history=deque(maxlen=200);self.history_index=None
        self.settings={'scan_y':.75,'target_x':.79,'gains':(6.5,0.,.8)}
        root=QWidget();self.setCentralWidget(root);layout=QVBoxLayout(root);layout.setContentsMargins(24,18,24,18);layout.setSpacing(12)
        title=QLabel('처음 만나는 자율주행');title.setStyleSheet('font-size:28px;font-weight:700;');layout.addWidget(title)
        layout.addWidget(QLabel('인지: 카메라 기록  →  판단: 영역·차선 찾기  →  제어: 오차가 0에 수렴하도록 조향값·속도값 조절'))
        nav=QHBoxLayout();self.buttons=[];self.nav_group=QButtonGroup(self)
        for i,(name,_,__) in enumerate(LESSONS):
            button=QPushButton(name.replace('&','&&'));button.setCheckable(True);button.toggled.connect(lambda checked,j=i:self.select_stage(j) if checked else None)
            self.nav_group.addButton(button);nav.addWidget(button);self.buttons.append(button)
        layout.addLayout(nav)
        self.description=QLabel();self.description.setWordWrap(True);self.description.setStyleSheet('font-size:21px;font-weight:600;');layout.addWidget(self.description)
        self.hint=QLabel();self.hint.setWordWrap(True);layout.addWidget(self.hint)
        viewrow=QHBoxLayout();self.compare=QPushButton('전후 나란히 보기');self.compare.setCheckable(True);self.compare.setChecked(True)
        self.compare.toggled.connect(self.render);viewrow.addWidget(self.compare)
        self.mode=QLabel();viewrow.addWidget(self.mode);viewrow.addStretch();layout.addLayout(viewrow)
        content=QHBoxLayout();self.pictures=[Picture() for _ in range(3)]
        for panel in self.pictures:content.addWidget(panel,1)
        self.error_view=ErrorView();content.addWidget(self.error_view,1);self.error_view.hide()
        self.pid_comparison=PIDComparison();content.addWidget(self.pid_comparison,1);self.pid_comparison.hide()
        self.controls=QGroupBox('직접 바꿔보기');self.controls.setFixedWidth(290)
        self.form=QVBoxLayout(self.controls);self.build_controls();content.addWidget(self.controls)
        layout.addLayout(content,1)
        self.notice=QLabel('준비 중 · 녹화 파일과 AI 모델을 불러오고 있어요.');self.notice.setWordWrap(True);layout.addWidget(self.notice)
        self.timeline=QSlider(Qt.Horizontal);self.timeline.setAccessibleName('재생 위치');self.timeline.setRange(0,1000)
        self.timeline.sliderPressed.connect(self.begin_seek);self.timeline.valueChanged.connect(self.seek)
        layout.addWidget(self.timeline)
        transport=QHBoxLayout();self.play=QPushButton('일시정지');self.play.clicked.connect(self.toggle_play)
        self.step=QPushButton('한 장 앞으로');self.step.clicked.connect(self.next_frame)
        self.restart=QPushButton('처음부터');self.restart.clicked.connect(self.rewind)
        self.speed=QComboBox();self.speed.addItems(['0.5배속','1배속','2배속']);self.speed.setCurrentIndex(1);self.speed.currentIndexChanged.connect(self.change_speed)
        self.clock=QLabel('0:00 / 0:00');self.clock.setMinimumWidth(220)
        for widget in (self.play,self.step,self.restart):transport.addWidget(widget)
        self.speed.hide();self.speed_buttons=[];self.speed_group=QButtonGroup(self)
        for i,name in enumerate(('0.5배속','1배속','2배속')):
            button=QPushButton(name);button.setCheckable(True);button.setChecked(i==1)
            button.toggled.connect(lambda checked,j=i:self.speed.setCurrentIndex(j) if checked else None)
            self.speed_group.addButton(button);self.speed_buttons.append(button);transport.addWidget(button)
        transport.addWidget(self.clock)
        transport.addStretch();close=QPushButton('수업 종료');close.clicked.connect(self.close);transport.addWidget(close);layout.addLayout(transport)
        self.play.setEnabled(False);self.step.setEnabled(False);self.restart.setEnabled(False);self.timeline.setEnabled(False)
        self.shortcut=QShortcut(QKeySequence('Space'),self);self.shortcut.activated.connect(self.toggle_play)
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(35)
        self.thread=QThread(self);self.worker=Worker(bag,device);self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.load);self.request.connect(self.worker.process);self.stop_worker.connect(self.worker.stop)
        self.worker.ready.connect(self.on_ready);self.worker.frame.connect(self.on_frame);self.worker.failed.connect(self.on_error)
        self.thread.finished.connect(self.worker.deleteLater);self.thread.finished.connect(self.worker_stopped)
        self.select_stage(0);self.resize(1400,850);self.setMinimumSize(1050,730);self.thread.start()

    def build_controls(self):
        self.color_box=QWidget();color=QVBoxLayout(self.color_box);color.setContentsMargins(0,0,0,0)
        self.space=QComboBox();self.space.addItems(list(SPACES));self.space.currentTextChanged.connect(self.load_sliders);self.space.hide()
        row=QHBoxLayout();self.space_buttons=[];self.space_group=QButtonGroup(self)
        for i,name in enumerate(SPACES):
            button=QPushButton(name);button.setCheckable(True);button.setChecked(i==0)
            button.toggled.connect(lambda checked,j=i:self.space.setCurrentIndex(j) if checked else None)
            self.space_group.addButton(button);self.space_buttons.append(button);row.addWidget(button)
        color.addLayout(row)
        self.sliders=[];self.slider_labels=[]
        for i in range(6):
            label=QLabel();slider=QSlider(Qt.Horizontal)
            slider.valueChanged.connect(lambda value,j=i:self.adjust_color(j,value))
            color.addWidget(label);color.addWidget(slider);self.sliders.append(slider);self.slider_labels.append(label)
        reset=QPushButton('색 범위 되돌리기');reset.clicked.connect(self.reset_color);color.addWidget(reset)
        self.form.addWidget(self.color_box)
        self.scan_box=QWidget();scanform=QFormLayout(self.scan_box)
        self.scan_sliders=[]
        for name,key in [('살펴볼 높이','scan_y'),('원하는 차선 위치','target_x')]:
            slider=QSlider(Qt.Horizontal);slider.setRange(10,95);slider.setValue(round(self.settings[key]*100));slider.setAccessibleName(name)
            slider.valueChanged.connect(lambda v,k=key:self.adjust_scan(k,v));scanform.addRow(name,slider);self.scan_sliders.append(slider)
        reset_scan=QPushButton('차선 기준 되돌리기');reset_scan.clicked.connect(self.reset_scan);scanform.addRow(reset_scan);self.form.addWidget(self.scan_box)
        self.pid_box=QWidget();pidform=QFormLayout(self.pid_box);self.gains=[]
        for title,value,maximum in [('P · 현재 차이에 반응',6.5,20.),('I · 쌓인 차이에 반응',0.,5.),('D · 움직임 억제',.8,5.)]:
            spin=QDoubleSpinBox();spin.setRange(0,maximum);spin.setSingleStep(.1);spin.setValue(value);spin.setAccessibleName(title)
            spin.valueChanged.connect(self.adjust_pid);pidform.addRow(title,spin);self.gains.append(spin)
        reset_pid=QPushButton('PID 기본값으로');reset_pid.clicked.connect(self.reset_pid);pidform.addRow(reset_pid)
        pidnote=QLabel('P · 지금 차이가 크면 더 세게\n\nI · 남아 있는 차이를 쌓아 보정\n\nD · 빠른 조향 움직임을 억제');pidnote.setWordWrap(True);pidform.addRow(pidnote)
        self.i_state=QLabel('I = 0 · 누적 보정 꺼짐');self.i_state.setWordWrap(True);pidform.addRow(self.i_state)
        self.form.addWidget(self.pid_box);self.form.addStretch();self.load_sliders()

    def select_stage(self,stage):
        self.stage=stage
        for i,button in enumerate(self.buttons):button.setChecked(i==stage)
        self.description.setText(LESSONS[stage][1]);self.hint.setText(LESSONS[stage][2])
        self.error_view.setVisible(stage==3 and self.compare.isChecked());self.pid_comparison.setVisible(stage==3)
        self.compare.setVisible(stage!=0);self.color_box.setVisible(stage==4);self.scan_box.setVisible(stage==2);self.pid_box.setVisible(stage==3)
        self.controls.setVisible(stage in (2,3,4));self.render()

    def load_sliders(self):
        space=self.space.currentText();_,channels,limits=SPACES[space]
        for i,button in enumerate(self.space_buttons):button.setChecked(i==self.space.currentIndex())
        for j,slider in enumerate(self.sliders):
            channel=j//2;side=j%2;value=self.bounds[space][side][channel]
            name=f'{channels[channel]} {"최솟값" if side==0 else "최댓값"}'
            slider.blockSignals(True);slider.setRange(0,limits[channel]);slider.setValue(value);slider.setAccessibleName(f'{space} {name}');slider.blockSignals(False)
            self.slider_labels[j].setText(f'{name}   {value}')
        if hasattr(self,'mode'):self.render()

    def adjust_color(self,j,value):
        space=self.space.currentText();channel=j//2;side=j%2
        self.bounds[space][side][channel]=value
        # 제약: 최솟값이 최댓값을 넘어 검은 화면만 남는 실수를 예방한다.
        if self.bounds[space][0][channel]>self.bounds[space][1][channel]:self.bounds[space][1-side][channel]=value
        self.load_sliders()

    def reset_color(self):
        space=self.space.currentText();self.bounds[space]=tuple(list(v) for v in DEFAULTS[space]);self.load_sliders()
    def adjust_scan(self,key,value):self.settings[key]=value/100;self.refresh_calculation()
    def reset_scan(self):
        for slider,value in zip(self.scan_sliders,(75,79)):slider.setValue(value)
    def adjust_pid(self):
        self.settings['gains']=tuple(spin.value() for spin in self.gains);self.refresh_calculation()
    def reset_pid(self):
        for spin,value in zip(self.gains,(6.5,0.,.8)):spin.setValue(value)
        self.refresh_calculation()
    def refresh_calculation(self):
        self.pending_reset=True
        if self.loaded and not self.busy:self.dispatch(self.current,True)

    def on_ready(self,times):
        self.times=times;self.loaded=True;self.restart.setEnabled(True);self.timeline.setEnabled(True);self.play.setEnabled(True)
        self.playing=True;self.anchor=time.monotonic();self.notice.setText('재생 중 · 먼저 단계를 선택하고, 일시정지해서 전후를 비교해 보세요.');self.dispatch(0,True)
    def dispatch(self,index,reset=False):
        if self.busy:return
        if reset:self.error_history.clear();self.history_index=None
        self.busy=True;self.requested_index=index;self.sent_generation=self.generation
        self.pending_reset=False
        self.request.emit(index,dict(self.settings),reset)
    def on_frame(self,data):
        self.busy=False
        if self.closing:return
        if self.sent_generation!=self.generation:return
        self.data=data;self.current=data['index']
        if self.history_index is not None and self.current<self.history_index:self.error_history.clear()
        point=(self.times[self.current],data['pid'].error)
        if self.history_index==self.current and self.error_history:self.error_history[-1]=point
        else:self.error_history.append(point)
        self.history_index=self.current;self.render()
        if not self.timeline.isSliderDown():
            self.timeline.blockSignals(True);self.timeline.setValue(round(self.times[self.current]/max(self.times[-1],.001)*1000));self.timeline.blockSignals(False)
        self.clock.setText(f'{self.format_time(self.times[self.current])} / {self.format_time(self.times[-1])} · {self.current+1}장')
        if self.pending_reset:self.dispatch(self.current,True)
    @staticmethod
    def format_time(value):return f'{int(value)//60}:{value%60:04.1f}'
    def on_error(self,message):
        self.playing=False;self.loaded=False;self.busy=False
        self.notice.setText('실행하지 못했어요: '+message+'  · 파일과 설치 상태를 확인한 뒤 다시 실행해 주세요.')
        for widget in (self.play,self.step,self.restart,self.timeline):widget.setEnabled(False)
    def playback_time(self):return self.position+(time.monotonic()-self.anchor)*[.5,1.,2.][self.speed.currentIndex()] if self.playing else self.position
    def tick(self):
        if not self.loaded or self.closing or self.busy:return
        if self.pending_reset:self.dispatch(self.current,True);return
        if not self.playing:return
        position=self.playback_time()
        if position>self.times[-1]:
            self.position=0.;self.anchor=time.monotonic();position=0.;self.pending_reset=True
            self.notice.setText('처음부터 다시 재생 중 · 같은 장면을 반복해서 살펴볼 수 있어요.')
        index=max(0,min(len(self.times)-1,bisect.bisect_right(self.times,position)-1))
        if index!=self.current or self.data is None or self.pending_reset:self.dispatch(index,self.pending_reset)
    def pause(self):
        if not self.loaded:return
        self.position=self.times[self.current];self.playing=False;self.generation+=1
        self.play.setText('재생');self.step.setEnabled(True);self.notice.setText('일시정지 · 같은 순간의 전후를 비교하거나 한 장씩 넘겨보세요.')
    def toggle_play(self):
        if not self.loaded:return
        if self.playing:self.pause()
        else:
            self.playing=True;self.position=self.times[self.current];self.anchor=time.monotonic();self.play.setText('일시정지');self.step.setEnabled(False);self.notice.setText('재생 중 · 녹화된 카메라와 조향 기록을 함께 보고 있어요.')
    def next_frame(self):
        if not self.loaded or self.busy:return
        index=min(self.current+1,len(self.times)-1);self.position=self.times[index];self.dispatch(index)
    def rewind(self):
        if not self.loaded:return
        self.generation+=1;self.current=0;self.position=0.;self.anchor=time.monotonic();self.pending_reset=True
        self.notice.setText('처음으로 이동했어요 · PID 계산도 초기화합니다.')
    def begin_seek(self):self.pause()
    def seek(self,*unused):
        if not self.loaded:return
        self.pause();self.position=self.timeline.value()/1000*self.times[-1]
        self.current=max(0,bisect.bisect_right(self.times,self.position)-1);self.pending_reset=True
    def change_speed(self):
        for i,button in enumerate(self.speed_buttons):button.setChecked(i==self.speed.currentIndex())
        if not self.loaded:return
        self.position=self.times[self.current];self.anchor=time.monotonic()
        self.notice.setText(f'{self.speed.currentText()} 선택 · '+('재생 중' if self.playing else '일시정지 중'))

    def render(self,*unused):
        if not hasattr(self,'mode'):return
        compare=self.compare.isChecked()
        self.error_view.setVisible(self.stage==3 and compare)
        self.compare.setText(('오차와 출력 함께 보기 ✓' if compare else '오차와 출력 함께 보기') if self.stage==3 else ('전후 나란히 보기 ✓' if compare else '전후 나란히 보기'))
        self.hint.setText(LESSONS[self.stage][2] if compare or self.stage==0 else '처리 후 결과를 크게 보고 있어요. 전후 나란히 보기를 누르면 원래 입력과 비교할 수 있어요.')
        self.mode.setText('세 단계의 처리 결과' if self.stage==0 else ('같은 순간 · 왼쪽 BEFORE / 오른쪽 AFTER' if compare else '처리 후 AFTER만 보는 중'))
        if self.stage==3:
            self.hint.setText(LESSONS[3][2] if compare else '출력만 보고 있어요. 오차와 출력 함께 보기를 누르면 오차 그래프도 볼 수 있어요.')
            self.mode.setText('모의 실험과 녹화 결과는 서로 다른 비교예요.' if compare else '녹화 입력의 출력 비교')
        if self.data is None:return
        data=self.data;frame=data['frame'];labels=data['labels'];scan=data['scan'];result=data['pid']
        if self.stage==3:
            self.error_view.display(self.error_history,data['gains'])
            self.pid_comparison.display(data['p_only'],result,data['gains'])
            self.i_state.setText('I = 0 · 누적 보정 꺼짐' if data['gains'][1]==0 else 'I 보정 켜짐 · 재생하면서 변화를 보세요.')
        mask=cv2.cvtColor(scan.mask,cv2.COLOR_GRAY2BGR)
        overlay=cv2.addWeighted(frame,.35,PALETTE[labels],.65,0)
        measured=frame.copy();h,w=frame.shape[:2]
        cv2.line(measured,(0,scan.y),(w-1,scan.y),(255,220,0),3)
        cv2.line(measured,(round(scan.target_x),0),(round(scan.target_x),h-1),(0,150,255),3)
        if scan.measured_x is not None:
            point=(round(scan.measured_x),scan.y);cv2.circle(measured,point,9,(0,255,255),-1)
            cv2.arrowedLine(measured,point,(round(scan.target_x),scan.y),(0,255,255),3)
        scan_caption='차선을 찾지 못했어요. 이 순간에는 핸들 힘을 0으로 표시해요.' if scan.error_px is None else f'주황 선: 원하는 위치 · 노란 점: 찾은 차선\n두 위치의 차이: {abs(scan.error_px):.0f} 픽셀'
        power=np.full_like(frame,(38,28,18));mid=w//2
        cv2.line(power,(mid,85),(mid,265),(170,170,170),3)
        length=round(abs(result.output)/150*260)
        if length:
            end=mid+(length if result.output>0 else -length)
            cv2.arrowedLine(power,(mid,175),(end,175),(80,210,100),24,tipLength=.2)
        direction='오른쪽' if result.output>0 else '왼쪽' if result.output<0 else '멈춤'
        power_caption=f'{direction} · 힘 {abs(result.output)} / 150 (PWM)\n실제 모터 대신 계산 결과만 보여줘요.'
        if result.target is None or result.angle is None:power_caption='차선 또는 조향 기록이 없어요.\n계산된 힘: 0 · 입력을 기다려요.'
        seg=('AFTER · 영역에 색 입히기',overlay,'빨강: 실선 · 노랑: 점선 · 파랑: 도로')
        scanpanel=('AFTER · 차선 위치 찾기',measured,scan_caption)
        pidpanel=('AFTER · 핸들을 움직일 힘',power,power_caption)
        if self.stage==0:panels=[seg,scanpanel,pidpanel]
        elif self.stage==1:panels=[('BEFORE · 카메라 원본',frame,'차량 앞 카메라가 녹화한 사진이에요.'),seg]
        elif self.stage==2:panels=[('BEFORE · 모델이 찾은 차선',mask,'흰색 부분이 오른쪽 실선이에요.'),scanpanel]
        elif self.stage==3:panels=[]
        elif self.stage==4:
            space=self.space.currentText();lo,hi=self.bounds[space];filtered_mask,filtered=apply_filter(frame,space,lo,hi)
            panels=[('BEFORE · 카메라 원본',frame,'오른쪽에서 남길 색의 범위를 선택하세요.'),(f'AFTER · {space} 색 필터',filtered,f'검정: 지워진 부분 · 색이 남은 부분: {np.count_nonzero(filtered_mask)/filtered_mask.size:.0%}')]
        else:
            tracking=track_lane(labels);after=frame.copy()
            for x0,y0,x1,y1,found in tracking.windows:cv2.rectangle(after,(x0,y0),(x1-1,y1-1),(0,220,0) if found else (0,160,255),3)
            if tracking.curve is not None:cv2.polylines(after,[tracking.curve],False,(255,0,255),4)
            panels=[('BEFORE · 모델이 찾은 차선',mask,'흰색 차선을 작은 창으로 따라가요.'),('AFTER · 창으로 차선 따라가기',after,'초록: 찾음 · 주황: 못 찾음 · 분홍: 연결한 선' if tracking.curve is not None else '연결할 점이 부족해요. 다른 장면도 살펴보세요.')]
        if self.stage!=0 and not compare:panels=panels[1:]
        for i,picture in enumerate(self.pictures):
            picture.setVisible(i<len(panels))
            if i<len(panels):picture.display(*panels[i])

    def closeEvent(self,event):
        if self.thread.isRunning():
            event.ignore()
            if not self.closing:
                self.closing=True;self.timer.stop();self.centralWidget().setEnabled(False)
                self.notice.setText('수업을 종료하고 있어요…');self.stop_worker.emit()
        else:event.accept()
    def worker_stopped(self):
        if self.closing:self.close()


def main(argv=None):
    parser=argparse.ArgumentParser(description='ORDA 통합 수업 창: bag 자동 재생')
    parser.add_argument('--bag',type=Path);parser.add_argument('--device',default='auto',choices=['auto','cpu','mps','cuda'])
    args=parser.parse_args(argv)
    try:bag=args.bag or find_bag()
    except Exception as exc:parser.error(str(exc))
    app=QApplication.instance() or QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    app.setStyleSheet('''QWidget {font-size:16px;color:#18304b;background:#f3f6fb;}
        QPushButton {padding:10px 14px;border:1px solid #c3cede;border-radius:8px;background:white;}
        QPushButton:hover {background:#e1edff;} QPushButton:checked {background:#215aca;color:white;border-color:#215aca;}
        QPushButton:disabled {color:#8895a6;background:#e9eef5;}
        QGroupBox {font-weight:600;border:1px solid #c3cede;border-radius:12px;margin-top:15px;padding:15px 10px 8px;}
        QGroupBox::title {subcontrol-origin:margin;left:12px;} QLabel {background:transparent;}
        QComboBox,QDoubleSpinBox {background:white;padding:6px;border:1px solid #c3cede;border-radius:6px;}
        QSlider::groove:horizontal {height:6px;background:#cad6e7;border-radius:3px;}
        QSlider::handle:horizontal {background:#215aca;width:20px;margin:-7px 0;border-radius:10px;}
        QSlider::sub-page:horizontal {background:#6c9ceb;border-radius:3px;}''')
    window=Classroom(bag,args.device);window.show();return app.exec()

if __name__=='__main__':main()
