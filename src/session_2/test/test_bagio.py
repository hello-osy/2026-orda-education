import csv
from pathlib import Path
import time

import numpy as np
from rosbags.rosbag2 import Reader
from session_2.bagio import BagArchive,BagWriter,CAMERA_TOPICS,Recorder,SCAN_TOPIC,extract_frames


def make_bag(path,frames=12):
    writer=BagWriter(path,[*CAMERA_TOPICS,SCAN_TOPIC])
    try:
        for i in range(frames):
            stamp=1_700_000_000_000_000_000+i*100_000_000
            for j,topic in enumerate(CAMERA_TOPICS):
                image=np.full((48,64,3),i+j*80,dtype=np.uint8)
                writer.write(topic,stamp+j*10_000_000,image)
            writer.write(SCAN_TOPIC,stamp,np.array([[10,0,1000],[20,90,2000],[0,10,0]]))
    finally:writer.close()
    return path


def test_standard_bag_roundtrip_and_seek(tmp_path):
    path=make_bag(tmp_path/'recording')
    with Reader(path) as reader:
        assert reader.message_count==36
        assert {c.msgtype for c in reader.connections}=={'sensor_msgs/msg/Image','sensor_msgs/msg/LaserScan'}
    bag=BagArchive(path)
    try:
        snap=bag.snapshot(.55,[*CAMERA_TOPICS,SCAN_TOPIC])
        assert snap[CAMERA_TOPICS[0]]['index']==5
        assert int(snap[CAMERA_TOPICS[1]]['data'][0,0,0])==85
        scan=snap[SCAN_TOPIC]['data']
        assert scan.shape==(2,3)
        assert sorted(scan[:,2])==[1000,2000]
        assert -270 in scan[:,1]  # ROS CCW -> clockwise display, same as +90.
        assert CAMERA_TOPICS[1] not in bag.snapshot(0,CAMERA_TOPICS)
        assert bag.snapshot(0,[CAMERA_TOPICS[0]])[CAMERA_TOPICS[0]]['index']==0
    finally:bag.close()


def test_extract_every_fifth_per_camera_and_no_overwrite(tmp_path):
    import pytest
    path=make_bag(tmp_path/'bag')
    output=tmp_path/'images'
    counts=extract_frames(path,output,CAMERA_TOPICS)
    assert list(counts.values())==[2,2]
    with (output/'manifest.csv').open() as f: rows=list(csv.DictReader(f))
    assert [int(row['frame_1based']) for row in rows]==[5,10,5,10]
    assert len(list(output.rglob('*.jpg')))==4
    with pytest.raises(FileExistsError):extract_frames(path,output,CAMERA_TOPICS)


def test_recorder_drains_queue_and_finalizes_metadata(tmp_path):
    r=Recorder(tmp_path/'bag',[CAMERA_TOPICS[0]])
    assert r.ready.wait(3)
    for _ in range(6):r.submit(CAMERA_TOPICS[0],time.monotonic(),np.zeros((8,8,3),np.uint8))
    r.stop();r.thread.join(5)
    assert not r.thread.is_alive() and not r.error
    assert r.counts[CAMERA_TOPICS[0]]==6
    with Reader(r.path) as reader:assert reader.message_count==6


def test_capture_with_korean_and_spaces_in_path(tmp_path):
    import cv2
    import json
    path = make_bag(tmp_path/'수업 녹화')
    output = tmp_path/'학습 사진'
    counts = extract_frames(path, output, CAMERA_TOPICS)
    for file in output.rglob('*.jpg'):
        decoded = cv2.imdecode(np.frombuffer(file.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape == (48, 64, 3)
    with (output/'manifest.csv').open(encoding='utf-8') as handle:
        rows = list(csv.DictReader(handle))
    assert all('수업 녹화' in row['source_bag'] for row in rows)
    assert json.loads((output/'summary.json').read_text(encoding='utf-8'))['counts'] == counts
