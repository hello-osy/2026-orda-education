"""실제 DDS/rosbag 반복 재생 회귀 검사. 프로젝트 및 ROS 환경 source 후 실행."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

ROOT=Path(__file__).resolve().parents[1]
STAGES=['segmentation','scan_line','pid','reference_color_filter','reference_sliding_window','pipeline_after']
OUT=ROOT/'output/validation';OUT.mkdir(parents=True,exist_ok=True)

def main():
    (OUT/'metrics.jsonl').write_text('')
    rclpy.init();node=Node('session_1_validation')
    counts={s:0 for s in STAGES};images={s:0 for s in STAGES};rewinds={s:0 for s in STAGES};last={};statuses={};errors=[]
    subscriptions=[];children=[];logs=[]
    def metric(stage,msg):
        m=json.loads(msg.data);counts[stage]+=1
        if stage in last and m['stamp']<last[stage]:rewinds[stage]+=1
        last[stage]=m['stamp']
        if m.get('steering_pwm') is not None and abs(m['steering_pwm'])>150:errors.append('PWM out of bounds')
        if 'status' in m:statuses.setdefault(stage,set()).add(m['status'])
        if counts[stage]==1 or counts[stage]%50==0:
            with (OUT/'metrics.jsonl').open('a') as f:f.write(json.dumps(m)+'\n')
    def image(stage,msg):
        assert msg.encoding=='bgr8' and len(msg.data)==msg.height*msg.step
        images[stage]+=1
    try:
        for s in STAGES:
            subscriptions.append(node.create_subscription(String,f'/session_1/{s}/metrics',lambda m,s=s:metric(s,m),10))
            subscriptions.append(node.create_subscription(Image,f'/session_1/{s}/image',lambda m,s=s:image(s,m),1))
            log=(OUT/f'{s}.log').open('w');logs.append(log)
            children.append(subprocess.Popen(['ros2','run','session_1',s+'_view','--headless','--device','cpu','--output',str(OUT/f'{s}.png'),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=log,start_new_session=True))
        startup=time.monotonic()+4
        while time.monotonic()<startup:rclpy.spin_once(node,timeout_sec=.05)
        log=(OUT/'bag.log').open('w');logs.append(log)
        children.append(subprocess.Popen(['ros2','bag','play',str(ROOT/'rosbag2_2026_08_05-11_29_45'),'--clock','--loop','--rate','5','--topics','/camera/high/image_raw','/arduino/steering_raw'],stdout=log,stderr=log,start_new_session=True))
        deadline=time.monotonic()+55
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if all(counts[s]>=50 and rewinds[s]>=1 and images[s]>=10 for s in STAGES):break
        for s in STAGES:
            if counts[s]<50 or rewinds[s]<1 or images[s]<10:errors.append(f'{s}: insufficient output / no rewind')
        for topic in ['/motor','/motor_control','/arduino/motor_command']:
            if node.count_publishers(topic):errors.append(f'unexpected publisher {topic}')
    finally:
        for p in children:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();errors.append('forced termination')
        for f in logs:f.close()
        node.destroy_node();rclpy.shutdown()
    if any(p.returncode!=0 for p in children):errors.append('nonzero process exit')
    result={'counts':counts,'images':images,'rewinds':rewinds,'statuses':{k:sorted(v) for k,v in statuses.items()},'exit_codes':[p.returncode for p in children],'errors':errors}
    (OUT/'ros-summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    assert not errors,result

if __name__=='__main__':main()
