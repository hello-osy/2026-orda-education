"""실제 모델 동시 호출 검사. 센서·모터를 열지 않고 합성 영상을 사용한다."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
import numpy as np

from session_2.driving import DrivingPipeline
from session_2.learning import Detector


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--device', default='mps', choices=['mps', 'cuda', 'cpu'])
    args = parser.parse_args()
    rng = np.random.default_rng(42)
    frames = [rng.integers(0, 256, (240, 320, 3), dtype=np.uint8) for _ in range(3)]
    with ThreadPoolExecutor(2) as executor:
        lane_future = executor.submit(DrivingPipeline, args.device)
        yolo_future = executor.submit(Detector, args.weights, args.device)
        lane, yolo = lane_future.result(), yolo_future.result()
        print(f'devices: PIDNet={lane.device}, YOLO={yolo.device}', flush=True)

        def run_lane():
            for i in range(args.frames):
                result = lane.process(frames[i % 3], time.monotonic())
                assert result['segmentation'].shape == (352, 640, 3)
            return args.frames

        def run_yolo():
            for i in range(args.frames):
                image, count, ms = yolo.predict(frames[i % 3])
                assert image.shape == (240, 320, 3) and ms >= 0
            return args.frames

        start = time.monotonic()
        lane_future = executor.submit(run_lane)
        yolo_future = executor.submit(run_yolo)
        print(f'completed: PIDNet={lane_future.result()}, YOLO={yolo_future.result()}, '
              f'seconds={time.monotonic()-start:.1f}', flush=True)


if __name__ == '__main__':
    main()
