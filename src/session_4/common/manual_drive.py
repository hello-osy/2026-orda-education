"""완성된 공통 도구: 카메라를 보며 키/버튼을 누르는 동안만 직접 운전한다."""
import time


class ManualKeys:
    def __init__(self):
        self.pressed = set()
        self.speed = 120
        self.angle = 18
        self.enabled = False

    def clear(self):
        self.pressed.clear()

    def command(self):
        if not self.enabled:
            return [0, 0]
        direction = int('forward' in self.pressed) - int('back' in self.pressed)
        steer = int('left' in self.pressed) - int('right' in self.pressed)
        return [steer * self.angle, direction * self.speed]


def create_window(node):
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from rcl_interfaces.srv import SetParameters, GetParameters
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int16MultiArray
    from session_3.image_utils import image_to_rgb
    from PySide6.QtCore import Qt, QTimer, QEvent
    from PySide6.QtGui import QImage, QPixmap
    from PySide6.QtWidgets import (QWidget, QLabel, QPushButton, QGridLayout,
                                   QVBoxLayout, QHBoxLayout, QSpinBox, QApplication)

    class ManualWindow(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle('ORDA · 직접 운전하기')
            self.resize(760, 690)
            self.keys = ManualKeys()
            self.last_image = -float('inf')
            self.claimed = False
            self.pending = None
            self.last_poll = 0.0
            self.closed = False
            self.publisher = node.create_publisher(Int16MultiArray, '/session4/manual_control', 1)
            self.mode_client = node.create_client(SetParameters, '/session4_sim_control/set_parameters')
            self.mode_reader = node.create_client(GetParameters, '/session4_sim_control/get_parameters')
            self.image_subscription = node.create_subscription(
                Image, '/camera/high/image_raw', self.image, qos_profile_sensor_data)
            layout = QVBoxLayout(self)
            title = QLabel('카메라를 보며 직접 운전해 보세요')
            title.setStyleSheet('font-size:22px;font-weight:bold')
            layout.addWidget(title)
            self.camera = QLabel('시뮬레이터 카메라 연결을 기다립니다.')
            self.camera.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.camera.setMinimumSize(640, 350)
            self.camera.setStyleSheet('background:#17212b;color:white;border-radius:6px')
            layout.addWidget(self.camera, 1)
            self.status = QLabel('시뮬레이터 연결 중…')
            layout.addWidget(self.status)
            layout.addWidget(QLabel('W/↑ 앞으로 · S/↓ 뒤로 · A/← 왼쪽 · D/→ 오른쪽 · Space 정지'))
            layout.addWidget(QLabel('키나 버튼을 누르는 동안 움직입니다. 창을 벗어나면 멈춥니다.'))
            row = QHBoxLayout()
            row.addWidget(QLabel('속도'))
            speed = QSpinBox()
            speed.setRange(30, 200)
            speed.setSingleStep(10)
            speed.setValue(self.keys.speed)
            speed.valueChanged.connect(lambda value: setattr(self.keys, 'speed', value))
            row.addWidget(speed)
            row.addWidget(QLabel('조향각'))
            angle = QSpinBox()
            angle.setRange(5, 30)
            angle.setValue(self.keys.angle)
            angle.valueChanged.connect(lambda value: setattr(self.keys, 'angle', value))
            row.addWidget(angle)
            row.addStretch()
            layout.addLayout(row)
            grid = QGridLayout()
            for label, action, position in [('↑ 앞으로', 'forward', (0, 1)),
                    ('← 왼쪽', 'left', (1, 0)), ('오른쪽 →', 'right', (1, 2)),
                    ('↓ 뒤로', 'back', (2, 1))]:
                button = QPushButton(label)
                button.setMinimumHeight(38)
                button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                button.pressed.connect(lambda key=action: self.keys.pressed.add(key))
                button.released.connect(lambda key=action: self.keys.pressed.discard(key))
                grid.addWidget(button, *position)
            stop = QPushButton('정지')
            stop.setMinimumHeight(38)
            stop.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            stop.pressed.connect(self.stop)
            grid.addWidget(stop, 1, 1)
            layout.addLayout(grid)
            self.setStyleSheet('QWidget{font-size:14px} QPushButton{padding:5px}')
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.tick)
            self.timer.start(30)
            self.bindings = {Qt.Key.Key_W: 'forward', Qt.Key.Key_Up: 'forward',
                Qt.Key.Key_S: 'back', Qt.Key.Key_Down: 'back', Qt.Key.Key_A: 'left',
                Qt.Key.Key_Left: 'left', Qt.Key.Key_D: 'right', Qt.Key.Key_Right: 'right'}
            # 버튼·설정칸에 초점이 있어도 이 창의 운전 키를 함께 처리한다.
            QApplication.instance().installEventFilter(self)

        def eventFilter(self, watched, event):
            kind = event.type()
            if kind in (QEvent.Type.ApplicationDeactivate, QEvent.Type.WindowDeactivate):
                self.stop()
            if kind in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) and self.isActiveWindow():
                key = event.key()
                if key == Qt.Key.Key_Space:
                    self.stop()
                    return True
                if key in self.bindings:
                    if not event.isAutoRepeat():
                        action = self.bindings[key]
                        if kind == QEvent.Type.KeyPress:
                            self.keys.pressed.add(action)
                        else:
                            self.keys.pressed.discard(action)
                    return True
            return super().eventFilter(watched, event)

        def image(self, message):
            try:
                rgb = np.ascontiguousarray(image_to_rgb(message))
                self.last_image = time.monotonic()
                picture = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0],
                                 QImage.Format.Format_RGB888).copy()
                self.camera.setPixmap(QPixmap.fromImage(picture).scaled(
                    self.camera.size(), Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation))
            except ValueError as error:
                self.status.setText(str(error))

        def stop(self):
            self.keys.clear()
            self.publisher.publish(Int16MultiArray(data=[0, 0]))

        def tick(self):
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if self.pending is not None and self.pending.done():
                response = self.pending.result()
                if self.claimed:
                    mode = response.values[0].string_value if response and response.values else ''
                    self.keys.enabled = mode == 'manual'
                else:
                    self.claimed = bool(response and all(value.successful for value in response.results))
                    self.keys.enabled = self.claimed
                self.pending = None
            if self.pending is None:
                if not self.claimed and self.mode_client.service_is_ready():
                    value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='manual')
                    self.pending = self.mode_client.call_async(SetParameters.Request(
                        parameters=[Parameter(name='mode', value=value)]))
                elif self.claimed and now - self.last_poll > .5 and self.mode_reader.service_is_ready():
                    self.pending = self.mode_reader.call_async(GetParameters.Request(names=['mode']))
                    self.last_poll = now
            if not self.keys.enabled or now - self.last_image > 1.0 or not self.isActiveWindow():
                self.keys.clear()
            command = self.keys.command()
            self.publisher.publish(Int16MultiArray(data=command))
            if not self.claimed:
                self.status.setText('시뮬레이터를 먼저 켜세요. 연결을 기다립니다.')
            elif not self.keys.enabled:
                self.status.setText('코드로 운전하는 중입니다. 직접 운전하려면 이 창을 닫고 다시 실행하세요.')
            elif now - self.last_image > 1.0:
                self.status.setText('카메라 연결을 기다립니다. 차량은 정지합니다.')
            else:
                self.status.setText(f'직접 운전 · 조향 {command[0]}도 · 속도 {command[1]}')

        def closeEvent(self, event):
            self.closed = True
            self.timer.stop()
            self.stop()
            QApplication.instance().removeEventFilter(self)
            event.accept()

    return ManualWindow()


def main(args=None):
    import signal
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from session_3.image_utils import prepare_qt
    prepare_qt()
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import QTimer
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication.instance() or QApplication([])
    node = Node('session4_manual_drive')
    window = create_window(node)
    old_handler = signal.signal(signal.SIGINT, lambda *_: window.close())
    heartbeat = QTimer()
    heartbeat.start(100)
    heartbeat.timeout.connect(lambda: None)
    try:
        window.show()
        app.exec()
    finally:
        if not window.closed:
            window.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_handler)


if __name__ == '__main__':
    main()
