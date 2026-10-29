"""ROS Image의 인코딩과 행 간격을 지키며 화면용 RGB 배열로 바꾼다."""
import numpy as np


def image_to_rgb(message):
    import cv2
    encoding = message.encoding.lower()
    channels = {'bgr8': 3, 'rgb8': 3, 'mono8': 1,
                'yuv422_yuy2': 2, 'yuv422': 2}.get(encoding)
    if channels is None:
        raise ValueError(f'지원하지 않는 영상 형식: {message.encoding}')
    width, height, step = int(message.width), int(message.height), int(message.step)
    if width <= 0 or height <= 0 or step < width * channels:
        raise ValueError('영상 크기 또는 step이 잘못되었습니다.')
    pixels = np.frombuffer(message.data, dtype=np.uint8)
    if pixels.size < height * step:
        raise ValueError('영상 데이터가 height × step보다 짧습니다.')
    packed = pixels[:height * step].reshape(height, step)[:, :width * channels]
    frame = packed.reshape(height, width, channels)
    if encoding == 'rgb8':
        return np.ascontiguousarray(frame)
    if encoding == 'bgr8':
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if encoding == 'mono8':
        return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2RGB)
    if width % 2:
        raise ValueError('YUV422 영상의 너비는 짝수여야 합니다.')
    code = cv2.COLOR_YUV2RGB_YUY2 if encoding == 'yuv422_yuy2' else cv2.COLOR_YUV2RGB_UYVY
    return cv2.cvtColor(frame, code)


def prepare_qt():
    """OpenCV wheel의 Qt5 경로가 PySide6의 Qt6 경로를 가리지 않게 한다."""
    import os
    from pathlib import Path
    import cv2
    bundled = Path(cv2.__file__).resolve().parent / 'qt'
    for name in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        value = os.environ.get(name)
        if value and Path(value).resolve().is_relative_to(bundled):
            os.environ.pop(name, None)
