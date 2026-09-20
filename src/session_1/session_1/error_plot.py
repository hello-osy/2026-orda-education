"""각도 그림 대신 오차의 시간 변화와 0 기준선을 보여주는 수업 카드."""
from PySide6.QtCore import Qt,QPointF
from PySide6.QtGui import QPainter,QColor,QPen,QPainterPath
from PySide6.QtWidgets import QWidget,QGroupBox,QVBoxLayout,QHBoxLayout,QPushButton,QLabel,QButtonGroup
from .error_response import simulate_response


class ErrorPlot(QWidget):
    def __init__(self):
        super().__init__();self.curves=[];self.setMinimumSize(230,160)
    def set_curves(self,curves):self.curves=curves;self.update()
    def paintEvent(self,event):
        p=QPainter(self);p.setRenderHint(QPainter.Antialiasing)
        left,top,right,bottom=40,15,self.width()-15,self.height()-32
        points=[point for curve in self.curves for point in curve if point[1] is not None]
        xmax=max((x for x,y in points),default=12.);xmin=min((x for x,y in points),default=0.)
        if xmax<=xmin:xmax=xmin+1
        extent=max(5.,max((abs(y) for x,y in points),default=20.)*1.15)
        def xy(x,y):return QPointF(left+(x-xmin)/(xmax-xmin)*(right-left),top+(extent-y)/(2*extent)*(bottom-top))
        p.fillRect(left,top,right-left,bottom-top,QColor('#f5f8fc'))
        p.setPen(QPen(QColor('#788ba3'),1,Qt.DashLine));zero=xy(xmin,0).y();p.drawLine(QPointF(left,zero),QPointF(right,zero))
        p.setPen(QColor('#18304b'));p.drawText(5,int(zero)+5,'0')
        p.drawText(left,bottom+24,'시간 →');p.drawText(right-50,bottom+24,f'{xmax-xmin:.1f}초')
        for i,curve in enumerate(self.curves):
            path=QPainterPath();started=False
            for x,y in curve:
                if y is None:started=False;continue
                point=xy(x,y)
                if not started:path.moveTo(point);started=True
                else:path.lineTo(point)
            p.setPen(QPen(QColor(('#2376ce','#13866c')[i%2]),3));p.drawPath(path)


class ErrorView(QGroupBox):
    def __init__(self):
        super().__init__('오차를 0으로 줄이기')
        layout=QVBoxLayout(self);row=QHBoxLayout();self.group=QButtonGroup(self)
        self.record=QPushButton('녹화된 오차');self.demo=QPushButton('수렴 모의 실험')
        for button in (self.record,self.demo):button.setCheckable(True);self.group.addButton(button);row.addWidget(button)
        self.demo.setChecked(True);self.demo.toggled.connect(self.refresh);layout.addLayout(row)
        self.summary=QLabel();self.summary.setWordWrap(True);layout.addWidget(self.summary)
        self.plot=ErrorPlot();layout.addWidget(self.plot,1)
        self.legend=QLabel();self.legend.setWordWrap(True);layout.addWidget(self.legend)
        self.note=QLabel();self.note.setWordWrap(True);layout.addWidget(self.note)
        self.history=[];self.gains=None;self.curves=[]
    def display(self,history,gains):
        self.history=list(history)
        if self.gains!=gains:self.gains=gains;self.curves=simulate_response(gains)
        self.refresh()
    def refresh(self,*unused):
        if self.gains is None:return
        if self.demo.isChecked():
            self.setTitle('교육용 모의 실험 · 오차가 줄어드는 과정')
            self.summary.setText('같은 오차 20에서 출발해요.\n0에 가까워지는지, 지나쳐 흔들리는지 보세요.')
            self.plot.set_curves(self.curves)
            self.legend.setText(f'파랑: P만 · 초록: PID\n12초 뒤 오차  P {self.curves[0][-1][1]:+.1f} / PID {self.curves[1][-1][1]:+.1f}')
            self.note.setText('오른쪽 P·I·D 값을 바꿔보세요.\n가상의 반응 예시이며, 실제 차량 결과는 아니에요.')
        else:
            self.setTitle('녹화된 오차 · 지금 목표에서 얼마나 멀까요?')
            error=self.history[-1][1] if self.history else None
            self.summary.setText('입력이 없어 오차를 계산할 수 없어요.' if error is None else f'현재 남은 오차 {error:+.1f} · 목표는 0')
            self.plot.set_curves([self.history]);self.legend.setText('파랑: 녹화 입력으로 계산한 오차 · 점선: 목표 0')
            self.note.setText('값을 바꿔도 녹화된 움직임은 그대로예요.\n빈 구간은 차선 또는 센서 기록이 없는 때예요.')
