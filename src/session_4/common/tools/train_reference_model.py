"""강사용: 수집한 Gazebo 전용 자료로 정답 모델을 학습한다."""
def main():
    from pathlib import Path
    import os,json,time,hashlib
    os.environ['YOLO_OFFLINE']='true'
    from session_4.dataset_tools import load_session2_module
    from session_4.training_tools import validate_training_dataset
    root=Path.cwd();data=root/'workspace/session_4/reference_dataset'
    seen=set();removed=[]
    for split in ['train','val']:
     for image in sorted((data/'images'/split).glob('*.png')):
      digest=hashlib.sha256(image.read_bytes()).hexdigest()
      if digest in seen:
       removed.append(str(image.relative_to(data)));image.unlink();(data/'labels'/split/(image.stem+'.txt')).unlink()
      else:seen.add(digest)
    (data/'deduplication.json').write_text(json.dumps(removed,indent=2))
    config,summary=validate_training_dataset(data/'data.yaml',root)
    print(summary,flush=True)
    load_session2_module('platform_support').prepare_learning_config(root/'workspace/session_4')
    from ultralytics import YOLO
    import torch
    print('device mps:',torch.backends.mps.is_available(),flush=True)
    model=YOLO(str(root/'yolov8n.pt'))
    start=time.monotonic()
    result=model.train(data=str(data/'data.yaml'),epochs=40,batch=8,imgsz=320,device='mps',workers=0,amp=False,plots=False,cache=False,seed=147,
     project=str(root/'workspace/session_4/reference_training'),name='answer',exist_ok=False,patience=12,
     fliplr=0.0,flipud=0.0,hsv_h=0.0,hsv_s=0.15,hsv_v=0.15,mosaic=0.3,close_mosaic=5,translate=0.05,scale=0.2)
    p=Path(model.trainer.best)
    (root/'workspace/session_4/reference_training/result.json').write_text(json.dumps(dict(best=str(p),elapsed=time.monotonic()-start,metrics=result.results_dict,dataset=summary),indent=2))
    print('FINISHED',p,flush=True)


if __name__ == "__main__":
    main()
