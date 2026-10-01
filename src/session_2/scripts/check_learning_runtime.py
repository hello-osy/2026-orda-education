"""합성 bag 및 학습 결과로 CPU/MPS 추론 경로를 검사. 모터 포트를 열지 않는다."""
import argparse
from pathlib import Path
import sys
import time
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[2]/'session_1')]
from session_2.bagio import BagArchive,CAMERA_TOPICS
from session_2.learning import Detector
from session_2.driving import DrivingPipeline


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--device',default='cpu');args=parser.parse_args()
    import json
    result=json.loads((args.root/'smoke_train'/'result.json').read_text(encoding='utf-8'))
    archive=BagArchive(args.root/'synthetic_bag')
    try:
        _,frame=archive.read(CAMERA_TOPICS[0],4)
        detector=Detector(result['best'],args.device)
        annotated,count,ms=detector.predict(frame,imgsz=64)
        assert annotated.shape==frame.shape
        pipeline=DrivingPipeline(args.device)
        output=pipeline.process(frame,time.monotonic())
        assert output['overlay'].shape==(352,640,3)
        print({'device':args.device,'yolo_ms':round(ms,1),'detections':count,'pidnet_target':output['target'],'motor_port_opened':False})
    finally:archive.close()


if __name__=='__main__':main()
