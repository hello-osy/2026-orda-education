"""소프트웨어 검증용 합성 데이터. 실제 주행 학습 데이터가 아니다."""
from pathlib import Path
import sys
import cv2
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from session_2.bagio import BagWriter,CAMERA_TOPICS,SCAN_TOPIC


def create(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    bag=root/'synthetic_bag'
    if not bag.exists():
        writer=BagWriter(bag,[*CAMERA_TOPICS,SCAN_TOPIC])
        try:
            for i in range(60):
                stamp=1_700_000_000_000_000_000+i*100_000_000
                for j,topic in enumerate(CAMERA_TOPICS):
                    frame=np.full((352,640,3),(30+j*30,35,45),np.uint8)
                    cv2.rectangle(frame,(80+i*4,100),(160+i*4,220),(40,190,240),-1)
                    cv2.putText(frame,f'SYNTHETIC CAM {j+1} / FRAME {i+1}',(20,45),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
                    writer.write(topic,stamp+j*1_000_000,frame)
                angles=np.arange(360)
                scan=np.column_stack((np.full(360,15),angles,2000+300*np.sin(np.deg2rad(angles*3+i))))
                writer.write(SCAN_TOPIC,stamp,scan)
        finally:writer.close()
    dataset=root/'synthetic_dataset'
    for split,count in [('train',4),('valid',2)]:
        images=dataset/split/'images';labels=dataset/split/'labels'
        images.mkdir(parents=True,exist_ok=True);labels.mkdir(exist_ok=True)
        for i in range(count):
            rng=np.random.default_rng(i+(100 if split=='valid' else 0))
            image=rng.integers(0,70,(64,64,3),dtype=np.uint8)
            cv2.rectangle(image,(16,16),(48,48),(200,200,200),-1)
            cv2.imwrite(str(images/f'{i}.jpg'),image)
            (labels/f'{i}.txt').write_text('0 0.5 0.5 0.5 0.5\n', encoding='utf-8')
    (dataset/'data.yaml').write_text(yaml.safe_dump({'train':'../train/images','val':'../valid/images','nc':1,'names':['test_box']}), encoding='utf-8')
    print(root.resolve())


if __name__=='__main__':create(sys.argv[1])
