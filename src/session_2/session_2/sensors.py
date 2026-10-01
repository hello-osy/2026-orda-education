"""센서 I/O는 별도 프로세스에서 실행한다. UI에는 최신 데이터만 전달한다."""
import queue
import sys
import time

import numpy as np
from .platform_support import camera_backend, camera_help


def publish(out, kind, payload):
    message = (kind, time.monotonic(), payload)
    try:
        out.put_nowait(message)
    except queue.Full:
        try:
            out.get_nowait()
        except queue.Empty:
            pass
        try:
            out.put_nowait(message)
        except queue.Full:
            pass


def camera_worker(out, stop, index, width, height, fps):
    import cv2
    cap = None
    out.cancel_join_thread()
    try:
        backend = camera_backend(cv2)
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            raise RuntimeError('카메라 열기 실패: 카메라 번호·USB 연결·다른 앱 사용 여부를 확인하세요. ' + camera_help())
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        last_good = time.monotonic()
        while not stop.is_set():
            ok, frame = cap.read()
            if not ok:
                # AVFoundation은 장치 초기화 직후 잠시 빈 프레임을 반환할 수 있다.
                if time.monotonic() - last_good < 2:
                    stop.wait(.05)
                    continue
                raise RuntimeError('영상 수신 실패: USB 연결 / 허브 전원 확인 후 다시 시작')
            last_good = time.monotonic()
            publish(out, 'data', frame)
    except Exception as exc:
        publish(out, 'error', str(exc))
    finally:
        if cap is not None:
            cap.release()


def lidar_worker(out, stop, port, baudrate):
    from rplidar import RPLidar
    lidar = None
    out.cancel_join_thread()
    try:
        lidar = RPLidar(port, baudrate=baudrate, timeout=1)
        info = lidar.get_info()
        health = lidar.get_health()
        publish(out, 'status', f"RPLIDAR {info['model']} · 상태 {health[0]}")
        if health[0] == 'Error':
            raise RuntimeError(f'라이다 하드웨어 오류: {health}')
        for scan in lidar.iter_scans(max_buf_meas=1000):
            if stop.is_set():
                break
            publish(out, 'data', np.asarray(scan, dtype=np.float32))
    except Exception as exc:
        publish(out, 'error', str(exc))
    finally:
        if lidar is not None:
            for cleanup in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                try:
                    cleanup()
                except Exception:
                    pass


def scan_xy(scan, limit_m, rotation_deg=0):
    """A1 각도 0도를 화면 위쪽, +90도를 오른쪽으로 표시. mm -> m."""
    scan = np.asarray(scan, dtype=float).reshape(-1, 3)
    valid = np.isfinite(scan).all(axis=1) & (scan[:, 0] > 0)
    valid &= (scan[:, 2] > 0) & (scan[:, 2] <= limit_m * 1000)
    angle = np.deg2rad(scan[valid, 1] + rotation_deg)
    distance = scan[valid, 2] / 1000
    return np.column_stack((distance * np.sin(angle), -distance * np.cos(angle)))


def serial_ports():
    from serial.tools.list_ports import comports
    return sorted(comports(), key=lambda p: p.device)


def suggested_port():
    # CP2102는 다른 장치일 수도 있으므로 UI에서 명시적으로 시작한다.
    candidates = [p.device for p in serial_ports() if p.vid == 0x10C4 and p.pid == 0xEA60]
    return candidates[0] if len(candidates) == 1 else ''
