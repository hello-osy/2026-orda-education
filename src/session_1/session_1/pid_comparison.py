"""제어 결과 비교: 같은 입력에서 P만 사용한 출력과 설정된 PID 출력을 표시.

막대는 각도가 아니라 PWM이다. 두 제어기의 실제 주행 응답은 시뮬레이션하지
않으므로 어느 쪽이 더 안정적이라는 평가는 표시하지 않는다.
"""
from PySide6.QtCore import Qt
from PySide6.QtGui import QPainter,QColor
from PySide6.QtWidgets import QWidget,QGroupBox,QVBoxLayout,QLabel,QHBoxLayout


class OutputBar(QWidget):
    def __init__(self,color):
        super().__init__();self.value=0;self.color=QColor(color);self.setMinimumHeight(36)
    def set_output(self,value):self.value=value;self.update()
    def paintEvent(self,event):
        painter=QPainter(self);painter.setRenderHint(QPainter.Antialiasing)
        width=self.width();mid=width/2;half=max(0,mid-8);y=7;height=20
        painter.setPen(Qt.NoPen);painter.setBrush(QColor('#e0e8f2'));painter.drawRoundedRect(8,y,width-16,height,8,8)
        extent=half*abs(self.value)/150
        painter.setBrush(self.color)
        painter.drawRoundedRect(int(mid if self.value>=0 else mid-extent),y,round(extent),height,5,5)
        painter.setPen(QColor('#52667c'));painter.drawLine(int(mid),3,int(mid),33)


class PIDComparison(QGroupBox):
    def __init__(self):
        super().__init__('녹화 입력 · P/PID 출력 비교')
        box=QVBoxLayout(self);box.setSpacing(6)
        self.values=[];self.bars=[]
        for title,color in [('P만 사용','#2376ce'),('PID 사용','#13866c')]:
            row=QHBoxLayout();label=QLabel(title);label.setStyleSheet('font-size:21px;font-weight:700;')
            value=QLabel();value.setAlignment(Qt.AlignRight|Qt.AlignVCenter);value.setStyleSheet(f'font-size:23px;font-weight:700;color:{color};')
            row.addWidget(label);row.addWidget(value);box.addLayout(row)
            bar=OutputBar(color);box.addWidget(bar);self.values.append(value);self.bars.append(bar)
        axis=QHBoxLayout()
        for name,alignment in [('← 왼쪽 −150',Qt.AlignLeft),('0',Qt.AlignCenter),('오른쪽 +150 →',Qt.AlignRight)]:
            label=QLabel(name);label.setAlignment(alignment);axis.addWidget(label)
        box.addLayout(axis)
        self.difference=QLabel();self.difference.setWordWrap(True);box.addWidget(self.difference)
        self.terms=QLabel();self.terms.setWordWrap(True);box.addWidget(self.terms)
        self.explanation=QLabel();self.explanation.setWordWrap(True);box.addWidget(self.explanation)
        box.addStretch()
        note=QLabel('같은 녹화 입력·같은 P값의 모터 명령(PWM) 비교\n녹화된 움직임은 바뀌지 않아요.')
        note.setWordWrap(True);box.addWidget(note)
    def display(self,p_only,pid,gains):
        for value,bar,result in zip(self.values,self.bars,(p_only,pid)):
            value.setText(f'{result.output:+d}' if result.output else '0');bar.set_output(result.output)
        self.difference.setText(f'출력 차이 (PID − P): {pid.output-p_only.output:+d}')
        self.terms.setText(f'PID 계산: P {pid.p:+.1f}  +  I {pid.i:+.1f}  +  D {pid.d:+.1f}')
        if pid.target is None or pid.angle is None:
            detail='차선 또는 조향 기록이 없어 출력을 멈췄어요.'
        elif abs(pid.error)<=1.:
            detail='오차가 허용 범위 안이라 두 출력 모두 0이에요.'
        elif gains[1]==0 and gains[2]==0:
            detail='I와 D가 0이라 P만 쓴 결과와 같아요.'
        elif p_only.output==pid.output:
            detail='보정이 작거나 출력 제한에 걸리면 두 값이 같을 수 있어요.'
        else:
            detail='I·D 보정으로 출력이 달라졌어요.'
        self.explanation.setText(detail+'\n두 출력 모두 최대 150 · 움직일 때 최소 40을 적용해요.')
