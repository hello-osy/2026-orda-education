import cv2
import numpy as np
import pytest
import yaml
from session_2.learning import validate_dataset


def dataset(tmp_path):
    for i,split in enumerate(['train','valid']):
        folder=tmp_path/split
        (folder/'images').mkdir(parents=True);(folder/'labels').mkdir()
        cv2.imwrite(str(folder/'images'/'sample.jpg'),np.full((32,32,3),i*100,np.uint8))
        (folder/'labels'/'sample.txt').write_text('0 0.5 0.5 0.3 0.3\n')
    path=tmp_path/'data.yaml'
    path.write_text(yaml.safe_dump({'train':'../train/images','val':'../valid/images','nc':1,'names':['car']}))
    return path


def test_roboflow_relative_paths_and_boxes(tmp_path):
    config,summary=validate_dataset(dataset(tmp_path))
    assert config['train']==str(tmp_path/'train'/'images')
    assert summary['train']['boxes']==1


def test_bad_labels_and_train_val_leakage(tmp_path):
    path=dataset(tmp_path)
    label=tmp_path/'valid'/'labels'/'sample.txt'
    label.write_text('5 0.5 0.5 0.3 0.3')
    with pytest.raises(ValueError,match='bbox'):validate_dataset(path)
    label.write_text('0 0.5 0.5 0.3 0.3')
    (tmp_path/'valid'/'images'/'sample.jpg').write_bytes((tmp_path/'train'/'images'/'sample.jpg').read_bytes())
    with pytest.raises(ValueError,match='동일 이미지'):validate_dataset(path)
