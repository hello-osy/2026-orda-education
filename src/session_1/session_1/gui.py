"""Qt 화면/슬라이더. OpenCV는 이미지 처리에만 사용한다.

macOS OpenCV HighGUI 창의 접근성 조회 시간 초과를 피하기 위해
표준 Qt 위젯과 이벤트 처리를 사용한다. ROS/판단 계산과 독립적이다.
"""
from collections import deque
import sys
import numpy as np
from PySide6.QtCore import Qt,QTimer,QEvent
from PySide6.QtGui import QImage,QPixmap,QShortcut,QKeySequence
from PySide6.QtWidgets import (QApplication,QMainWindow,QWidget,QLabel,QVBoxLayout,
                               QHBoxLayout,QPushButton,QSlider,QTabWidget,QFormLayout,QSizePolicy)


class Viewer(QMainWindow):
    def __init__(self,title,color_filter=None):
        self.app=QApplication.instance() or QApplication([])
        super().__init__()
        self.keys=deque();self.closed=False;self.pixmap=None;self.shortcuts=[]
        self.setWindowTitle(title)
        root=QWidget();self.setCentralWidget(root);layout=QVBoxLayout(root)
        toolbar=QHBoxLayout();layout.addLayout(toolbar)
        self.pause_button=QPushButton('일시정지 (Space)')
        self.pause_button.clicked.connect(lambda:self.keys.append(32));toolbar.addWidget(self.pause_button)
        reset=QPushButton('PID 초기화 (R)');reset.clicked.connect(lambda:self.keys.append(ord('r')));toolbar.addWidget(reset)
        close=QPushButton('종료 (Q / Esc)');close.clicked.connect(lambda:self.keys.append(ord('q')));toolbar.addWidget(close)
        toolbar.addStretch()
        self.state=QLabel('실행 중');toolbar.addWidget(self.state)
        body=QHBoxLayout();layout.addLayout(body,1)
        self.image=QLabel('카메라 프레임 수신 대기 중')
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(400,240)
        self.image.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.image.setStyleSheet('background: #181818; color: white;')
        body.addWidget(self.image,1)
        self.sliders={}
        if color_filter is not None:
            from .reference_color_filter_view import SPACES
            tabs=QTabWidget();tabs.setFixedWidth(320);body.addWidget(tabs)
            for space,(_,channels,limits) in SPACES.items():
                page=QWidget();form=QFormLayout(page)
                explanation=QLabel('참고자료 · 각 채널 범위에 포함된 픽셀만 남깁니다.');explanation.setWordWrap(True);form.addRow(explanation)
                for index,(channel,maximum) in enumerate(zip(channels,limits)):
                    for side,label in enumerate(('min','max')):
                        name=f'{space} {channel} {label}'
                        row=QWidget();line=QVBoxLayout(row);line.setContentsMargins(0,0,0,0)
                        value=QLabel(str(color_filter.bounds[space][side][index]))
                        value.setAccessibleDescription(name+' value')
                        slider=QSlider(Qt.Horizontal);slider.setRange(0,maximum)
                        slider.setAccessibleName(name);slider.setObjectName(name)
                        slider.setValue(color_filter.bounds[space][side][index])
                        def changed(v,s=space,i=index,k=side,out=value):
                            color_filter.set_bound(s,i,k,v);out.setText(str(v))
                        slider.valueChanged.connect(changed)
                        line.addWidget(value);line.addWidget(slider)
                        form.addRow(f'{channel} {label}',row);self.sliders[name]=slider
                tabs.addTab(page,space)
        self.status=QLabel('입력 대기');layout.addWidget(self.status)
        self.filter_status=QLabel('');layout.addWidget(self.filter_status)
        for sequence,key in [('Space',32),('Escape',27)]:
            shortcut=QShortcut(QKeySequence(sequence),self)
            shortcut.activated.connect(lambda k=key:self.keys.append(k));self.shortcuts.append(shortcut)
        self.resize(1380 if color_filter else 1280,850 if color_filter else 720)
        self.app.installEventFilter(self)
        self.show()

    def eventFilter(self,watched,event):
        if event.type()==QEvent.KeyPress:
            # 한글 입력 상태에서도 실제 R/Q 위치의 키가 작동하도록 macOS keycode 처리.
            native=event.nativeVirtualKey() if sys.platform=='darwin' else -1
            if event.key()==Qt.Key_R or native==15:
                self.keys.append(ord('r'));return True
            if event.key()==Qt.Key_Q or native==12:
                self.keys.append(ord('q'));return True
        return super().eventFilter(watched,event)

    def show_frame(self,canvas,metrics,count):
        rgb=np.ascontiguousarray(canvas[:,:,::-1])
        image=QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format_RGB888).copy()
        self.pixmap=QPixmap.fromImage(image);self._fit()
        self.status.setText(f"프레임 {count} | 입력 시각 {metrics['stamp']:.3f} | {metrics['processing_ms']:.1f} ms | {metrics['device']}")
        if 'filters' in metrics:
            self.filter_status.setText(' | '.join(f"{name}: {m['pixels']} px" for name,m in metrics['filters'].items()))

    def _fit(self):
        if self.pixmap is not None:
            self.image.setPixmap(self.pixmap.scaled(self.image.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))

    def resizeEvent(self,event):
        super().resizeEvent(event)
        if hasattr(self,'image'):self._fit()

    def poll(self):
        if self.closed:return ord('q')
        return self.keys.popleft() if self.keys else -1

    def run(self,steps):
        """정식 Qt 이벤트 루프에서 한 프레임씩 처리해 macOS AX 요청도 서비스한다."""
        failures=[]
        timer=QTimer(self)
        def advance():
            try: next(steps)
            except StopIteration: self.app.quit()
            except BaseException as exc:
                failures.append(exc);self.app.quit()
        timer.timeout.connect(advance);timer.start(10)
        try: self.app.exec()
        finally:
            timer.stop();steps.close()
        if failures: raise failures[0]

    def set_paused(self,paused):
        self.state.setText('일시정지 · 색 필터 조절 가능' if paused else '실행 중')
        self.pause_button.setText('재개 (Space)' if paused else '일시정지 (Space)')

    def mark_reset(self):
        self.state.setText('PID 초기화 완료')

    def closeEvent(self,event):
        self.closed=True;event.accept()

    def shutdown(self):
        self.app.removeEventFilter(self)
        self.close()
