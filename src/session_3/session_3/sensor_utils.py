"""Session 2의 센서 읽기 방식을 ROS 발행용 프로세스로 분리한다."""
import math
import multiprocessing as mp
import queue
import sys
import time

import numpy as np


def publish_latest(out, kind, value):
    try:
        out.put_nowait((kind, time.monotonic(), value))
    except queue.Full:
        try:
            out.get_nowait()
        except queue.Empty:
            pass
        try:
            out.put_nowait((kind, time.monotonic(), value))
        except queue.Full:
            pass


def camera_worker(out, stop, device, width, height, fps):
    import cv2
    camera = None
    out.cancel_join_thread()
    try:
        backend = cv2.CAP_AVFOUNDATION if sys.platform == 'darwin' else (
            cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_V4L2)
        camera = cv2.VideoCapture(device, backend)
        if not camera.isOpened():
            raise RuntimeError(f'카메라 {device} 열기 실패: 장치 번호, 권한, 다른 앱의 사용 여부를 확인하세요.')
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        camera.set(cv2.CAP_PROP_FPS, fps)
        last_good = time.monotonic()
        reported_size = False
        while not stop.is_set():
            started = time.monotonic()
            ok, frame = camera.read()
            if stop.is_set():
                break
            if ok and frame is not None and frame.size:
                last_good = time.monotonic()
                captured_height, captured_width = frame.shape[:2]
                # 일부 카메라는 set() 크기를 무시한다. 네트워크 발행 크기는 항상 맞춘다.
                if (captured_width, captured_height) != (width, height):
                    method = cv2.INTER_AREA if captured_width >= width and captured_height >= height else cv2.INTER_LINEAR
                    frame = cv2.resize(frame, (width, height), interpolation=method)
                if not reported_size:
                    publish_latest(out, 'status', f'카메라 {device}: 입력 {captured_width}×{captured_height}'
                                   f' → 발행 {width}×{height} · BGR8 · 최대 {fps:g} FPS')
                    reported_size = True
                publish_latest(out, 'data', frame)
            elif time.monotonic() - last_good > 10:
                raise RuntimeError('10초 동안 카메라 영상 없음: USB 연결과 카메라 권한을 확인하세요.')
            stop.wait(max(0.005, 1.0 / fps - (time.monotonic() - started)))
    except Exception as exc:
        publish_latest(out, 'error', str(exc))
    finally:
        if camera is not None:
            camera.release()


def lidar_worker(out, stop, port, baudrate):
    from rplidar import RPLidar, RPLidarException

    class TimedRPLidar(RPLidar):
        def _read_response(self, dsize):
            # 0.9.5 원본의 inWaiting() 반복은 timeout을 지정해도 끝나지 않는다.
            # pyserial의 제한 시간 안에서 읽고, 짧은 응답은 오류로 알린다.
            if stop.is_set():
                raise InterruptedError('라이다 종료 요청')
            data = self._serial.read(dsize)
            if stop.is_set():
                raise InterruptedError('라이다 종료 요청')
            if len(data) != dsize:
                raise RPLidarException(f'라이다 응답 시간 초과: {dsize}바이트 중 {len(data)}바이트 수신')
            return data

    lidar = None
    out.cancel_join_thread()
    try:
        if stop.is_set():
            return
        lidar = TimedRPLidar(port, baudrate=baudrate, timeout=3)
        # 이전 실행의 스캔/모터 상태와 남은 바이트를 먼저 정리한다.
        lidar.stop()
        lidar.stop_motor()
        lidar.clean_input()
        info = lidar.get_info()
        if not isinstance(info, dict):
            raise RuntimeError(f'라이다 장치 정보 응답 오류: {info}')
        health = lidar.get_health()
        if (not isinstance(health, (tuple, list)) or len(health) != 2
                or health[0] not in ('Good', 'Warning', 'Error')):
            raise RuntimeError(f'라이다 상태 응답 오류: {health}')
        if health[0] == 'Error':
            raise RuntimeError(f'라이다 상태 오류: {health}')
        lidar.start_motor()
        if stop.wait(3.0) or stop.is_set():
            return
        for scan in lidar.iter_scans(max_buf_meas=3000):
            if stop.is_set():
                break
            publish_latest(out, 'data', scan)
    except InterruptedError:
        pass
    except Exception as exc:
        publish_latest(out, 'error', str(exc))
    finally:
        if lidar is not None:
            for cleanup in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                try:
                    cleanup()
                except Exception:
                    pass


class SensorProcess:
    """블로킹 USB 읽기가 ROS 콜백을 막지 않도록 spawn 프로세스를 사용한다."""
    def __init__(self, worker, *args):
        context = mp.get_context('spawn')
        self.out = context.Queue(maxsize=2)
        self.stop = context.Event()
        self.process = context.Process(target=worker, args=(self.out, self.stop, *args), daemon=True)
        self.process.start()

    def read(self):
        messages = []
        while True:
            try:
                messages.append(self.out.get_nowait())
            except queue.Empty:
                return messages

    def close(self):
        self.stop.set()
        # 최대 3초 센서 read와 후속 모터 정리가 끝날 시간을 준다.
        self.process.join(timeout=3.5)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=0.5)
        self.out.close()


def scan_ranges(scan, mount_yaw_deg=0.0, range_min=0.15, range_max=12.0):
    """원시 0°=앞, 시계방향 A1 값을 ROS 0°=앞, 반시계방향으로 변환.

    Session 2 bag의 180° 이동된 관례와 달리 Session 3은 시뮬레이터와
    같은 전방 기준을 쓴다. 센서 장착 회전은 mount_yaw_deg로 보정한다.
    1도 간격 [-180, 179]로 묶으며 빈 칸은 inf, 중복은 가까운 거리다.
    """
    if not math.isfinite(mount_yaw_deg):
        raise ValueError('라이다 장착 각도는 유한한 값이어야 합니다.')
    ranges = np.full(360, np.inf, dtype=np.float32)
    samples = np.asarray(scan, dtype=float).reshape(-1, 3)
    valid = np.isfinite(samples).all(axis=1) & (samples[:, 0] > 0)
    for quality, angle, distance_mm in samples[valid]:
        distance = distance_mm / 1000.0
        if range_min <= distance <= range_max:
            index = int(round((-angle + mount_yaw_deg + 180.0) % 360.0)) % 360
            ranges[index] = min(ranges[index], distance)
    return ranges.tolist()
