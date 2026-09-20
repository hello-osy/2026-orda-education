"""판단 1 / segmentation: BGR 카메라 영상에서 의미별 픽셀을 추출한다.

인지(센서 수신)는 runtime.py, 제어는 화면/교육 토픽 출력만 수행한다.
PIDNet이라는 신경망 이름과 마지막 단계의 PID 제어기는 서로 다르다.
"""
import json
from pathlib import Path
import cv2
import numpy as np
import torch
from .vendor.pidnet import get_pred_model

# 모델의 dataset_info 순서에 맞춘 BGR 색. 학습 입력이 아니라 표시용이다.
PALETTE = np.array([[30,30,30], [235,110,60], [40,40,235],
                    [0,195,255], [70,215,40], [235,220,0]], dtype=np.uint8)

class Segmenter:
    def __init__(self, model_path='', device='auto'):
        local = Path(__file__).resolve().parents[1] / 'models'
        if not local.exists():
            from ament_index_python.packages import get_package_share_directory
            local = Path(get_package_share_directory('session_1')) / 'models'
        path = Path(model_path).expanduser() if model_path else local / 'lane_pidnet_s.pt'
        # 커스텀 가중치는 동일 폴더의 메타데이터와 반드시 함께 사용한다.
        info = json.loads((path.parent / 'dataset_info.json').read_text())
        self.class_names = info['class_names']
        expected = ['background','road','lane_solid','lane_dashed','green_mat','light_gray']
        if self.class_names != expected:
            raise ValueError('이 수업은 6클래스 PIDNet 모델을 사용합니다. dataset_info.json 확인')
        if device == 'auto':
            device = 'cuda' if torch.cuda.is_available() else ('mps' if torch.backends.mps.is_available() else 'cpu')
        self.device = device
        torch.set_num_threads(min(4, torch.get_num_threads()))
        self.model = get_pred_model('pidnet-s', len(self.class_names))
        state = torch.load(path, map_location='cpu', weights_only=True)
        state = state.get('state_dict', state)
        # 학습 wrapper 접두사만 제거. 누락된 가중치로 무작위 추론하지 않는다.
        cleaned = {}
        for key, value in state.items():
            for prefix in ('module.', 'model.'):
                if key.startswith(prefix): key = key[len(prefix):]
            cleaned[key] = value
        wanted = self.model.state_dict()
        missing = [k for k,v in wanted.items() if k not in cleaned or cleaned[k].shape != v.shape]
        if missing: raise ValueError(f'가중치 불일치: {missing[:5]}')
        self.model.load_state_dict({k:cleaned[k] for k in wanted}, strict=True)
        self.model.to(device).eval()  # CPU/MPS/CUDA 모두 float32: 수업의 재현성 우선

    def predict(self, frame):
        # 판단 / segmentation: 학습과 같은 BGR→RGB, ImageNet 정규화.
        rgb = frame[:,:,::-1].astype(np.float32) / 255.0
        rgb = (rgb - np.array([.485,.456,.406],np.float32)) / np.array([.229,.224,.225],np.float32)
        tensor = torch.from_numpy(rgb.transpose(2,0,1).copy())[None].to(self.device)
        with torch.inference_mode():
            logits = self.model(tensor)
            logits = torch.nn.functional.interpolate(logits, size=frame.shape[:2], mode='bilinear', align_corners=True)
            return logits.argmax(1)[0].cpu().numpy().astype(np.uint8)

def visualize(frame, labels):
    """BEFORE 원본 / AFTER 모델의 픽셀 분류. 같은 프레임끼리 비교한다."""
    from .drawing import panel, stack
    after = cv2.addWeighted(frame,.35,PALETTE[labels],.65,0)
    legend = ['0 background / 1 road', '2 solid (red) / 3 dashed (yellow)',
              '4 green mat / 5 light gray']
    return stack(panel(frame,'BEFORE: camera BGR',['Input: camera pixels']),
                 panel(after,'AFTER: PIDNet classes',legend))

def main(args=None):
    from .runtime import run
    run('segmentation', args)

if __name__ == '__main__': main()
