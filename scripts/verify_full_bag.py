"""전체 bag 카메라 프레임을 누락 없이 처리하는 오프라인 검사(실제 PIDNet)."""
from pathlib import Path
import json
import sqlite3
import time
import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from session_1.runtime import decode_image
from session_1.segmentation_view import Segmenter,visualize
from session_1.scan_line_view import scan_line
from session_1.reference_sliding_window_view import track_lane
from session_1.reference_color_filter_view import ColorFilterView
from session_1.pid_view import PID,Feedback,History
from session_1.pipeline_after_view import visualize as after_view

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/validation'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    bag=ROOT/'rosbag2_2026_08_05-11_29_45/rosbag2_2026_08_05-11_29_45_0.db3'
    db=sqlite3.connect(f'file:{bag}?mode=ro',uri=True)
    topics=dict(db.execute('select name,id from topics'))
    camera=topics['/camera/high/image_raw'];feedback_id=topics['/arduino/steering_raw']
    expected=db.execute('select count(*) from messages where topic_id=?',(camera,)).fetchone()[0]
    segmenter=Segmenter(device='cpu');pid=PID();feedback=Feedback();history=History();filters=ColorFilterView()
    frames=0;missing=0;tracking=0;feedback_count=0;classes=set();pwm_min=150;pwm_max=-150;t0=time.monotonic();ms=[]
    for topic_id,timestamp,data in db.execute('select topic_id,timestamp,data from messages where topic_id in (?,?) order by timestamp',(camera,feedback_id)):
        if topic_id==feedback_id:
            feedback.update(deserialize_message(data,Int16).data,timestamp/1e9,timestamp/1e9);feedback_count+=1;continue
        msg=deserialize_message(data,Image);frame=decode_image(msg)
        frame=cv2.resize(frame,(640,352))
        begin=time.monotonic();labels=segmenter.predict(frame);ms.append((time.monotonic()-begin)*1000)
        assert labels.shape==frame.shape[:2] and labels.dtype==np.uint8
        classes.update(int(x) for x in np.unique(labels))
        scan=scan_line(labels);track=track_lane(labels)
        missing+=int(scan.error_px is None);tracking+=int(track.curve is not None)
        _,color_metrics=filters.render(frame)
        assert all(0<=m['pixels']<=640*352 for m in color_metrics.values())
        target=None if scan.error_px is None else float(np.clip(-scan.error_px/130*45,-45,45))
        fresh=feedback.received is not None and timestamp/1e9-feedback.received<=.5
        result=pid.update(target,feedback.angle if fresh else None,feedback.velocity,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
        assert abs(result.output)<=150
        pwm_min=min(pwm_min,result.output);pwm_max=max(pwm_max,result.output)
        history.add(scan.error_px,result);frames+=1
        if frames%300==0 or frames==expected:
            canvas=after_view(frame,labels,scan,result,history,'RECORDED A1')
            cv2.imwrite(str(OUT/f'full-bag-{frames:04d}.png'),canvas)
            print(f'{frames}/{expected} frames, {time.monotonic()-t0:.1f}s',flush=True)
    assert frames==expected and frames>0
    report={'frames':frames,'expected':expected,'feedback_messages':feedback_count,'scan_missing':missing,'sliding_curve_frames':tracking,'class_ids':sorted(classes),'pwm_range':[pwm_min,pwm_max],'inference_mean_ms':float(np.mean(ms)),'inference_p95_ms':float(np.percentile(ms,95)),'elapsed_sec':time.monotonic()-t0}
    db.close();(OUT/'full-bag-summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
