"""강사용: Gazebo 영상과 색 마스크의 임시 라벨로 정답 모델 학습 자료를 수집한다.
자동 생성 라벨은 samples와 겹침을 시각적으로 확인한 뒤 사용한다.
기존 데이터셋은 덮어쓰지 않는다. 프로젝트 루트에서 실행한다.
"""
def main():
    from pathlib import Path
    import os,sys,json,math,random,time,subprocess,signal,shutil
    import cv2,numpy as np,rclpy,yaml
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
    from session_3.image_utils import image_to_rgb
    from session_4.yolo_node import color_detections
    root=Path.cwd();out=root/'workspace/session_4/reference_dataset'
    if out.exists():raise RuntimeError('dataset already exists')
    out.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='88',GZ_PARTITION='session4_reference_dataset',ROS_LOG_DIR=str(out/'ros_logs'))
    rclpy.init(args=[],domain_id=88);n=Node('reference_data_capture');frames=[0,None]
    def received(m):frames[0]+=1;frames[1]=image_to_rgb(m)
    sub=n.create_subscription(Image,'/camera/high/image_raw',received,qos_profile_sensor_data)
    log=(out/'simulator.log').open('w')
    p=subprocess.Popen([sys.executable,shutil.which('ros2'),'launch','session_4','simulator.launch.py','gui:=false','partition:=session4_reference_dataset','signal_mode:=red'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    # ros2cli has no module entry point on some source installations.
    if p.poll() is not None:raise RuntimeError('sim launcher failed')
    client=n.create_client(SetParameters,'/session4_traffic_signal/set_parameters')
    def wait(cond,timeout=40):
     end=time.monotonic()+timeout
     while not cond():
      if p.poll() is not None:raise RuntimeError('simulator exited')
      if time.monotonic()>end:raise TimeoutError('capture timeout')
      rclpy.spin_once(n,timeout_sec=.04)
    def pose(x,y,yaw):
     req=f'name:"session4_car" position{{x:{x} y:{y} z:0.015}} orientation{{z:{math.sin(yaw/2)} w:{math.cos(yaw/2)}}}'
     r=subprocess.run(['gz','service','-s','/world/session4_track/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','2000','--req',req],capture_output=True,text=True)
     if 'data: true' not in r.stdout:raise RuntimeError(r.stdout+r.stderr)
     old=frames[0];wait(lambda:frames[0]>=old+3,10)
    def phase(color):
     pose(3.5,-5.75,0)
     wait(lambda:client.service_is_ready())
     req=SetParameters.Request(parameters=[Parameter(name='mode',value=ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=color))])
     fut=client.call_async(req);wait(fut.done)
     wait(lambda:any(d['label']==color for d in color_detections(frames[1])['detections']),20)
    def labels(rgb):
     h,w=rgb.shape[:2];res=[]
     # Active lamps are separated by their saturated colors. Exclude road below horizon.
     packet=color_detections(rgb)
     for d in packet['detections']:
      if d['label']=='cone':continue
      box=d['bbox'];res.append((['red','yellow','green'].index(d['label']),box))
     hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
     orange=((hsv[:,:,0]>=6)&(hsv[:,:,0]<23)&(hsv[:,:,1]>125)&(hsv[:,:,2]>80)).astype('uint8')
     # Bridge white stripes within each cone; horizontal closing is intentionally narrow.
     orange=cv2.morphologyEx(orange,cv2.MORPH_CLOSE,np.ones((19,3),np.uint8))
     contours,_=cv2.findContours(orange,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
     for c in contours:
      x,y,bw,bh=cv2.boundingRect(c)
      if cv2.contourArea(c)>=6 and bw>=3 and bh>=5 and bh/bw>.75:
       res.append((3,[x,y,x+bw,y+bh]))
     return [(i,[max(0,x1),max(0,y1),min(w,x2),min(h,y2)]) for i,(x1,y1,x2,y2) in res]
    meta=[];counts={}
    try:
     wait(lambda:frames[0]>=4,90)
     for split,seed,num in [('train',531,70),('val',923,23)]:
      rng=random.Random(seed);(out/'images'/split).mkdir(parents=True);(out/'labels'/split).mkdir(parents=True)
      idx=0;counts[split]=[0]*4
      for color in ['red','yellow','green']:
       phase(color)
       for k in range(num):
        if k<num*.72:x=rng.uniform(-4.5,4.7);y=-5.75+rng.uniform(-.28,.28);yaw=rng.uniform(-.16,.16)
        elif k<num*.88:x=rng.uniform(-6,3);y=-5.75+rng.uniform(-.2,.2);yaw=rng.uniform(-.45,.45)
        else:x=rng.uniform(-7,7);y=5.75+rng.uniform(-.22,.22);yaw=math.pi+rng.uniform(-.12,.12)
        pose(x,y,yaw);rgb=frames[1].copy();rows=labels(rgb);name=f'{color}_{idx:04d}'
        cv2.imwrite(str(out/'images'/split/(name+'.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        text=[]
        for cls,(x1,y1,x2,y2) in rows:
         counts[split][cls]+=1;text.append(f'{cls} {(x1+x2)/640:.7f} {(y1+y2)/480:.7f} {(x2-x1)/320:.7f} {(y2-y1)/240:.7f}')
        (out/'labels'/split/(name+'.txt')).write_text('\n'.join(text)+'\n')
        meta.append(dict(split=split,name=name,phase=color,pose=[x,y,yaw],boxes=rows));idx+=1
       print(split,color,idx,counts[split],flush=True)
     (out/'data.yaml').write_text(yaml.safe_dump(dict(path=str(out),train='images/train',val='images/val',names=['red','yellow','green','cone'])))
     (out/'provenance.json').write_text(json.dumps(dict(source='Gazebo camera; color-mask pseudo labels reviewed visually',samples=meta,counts=counts),indent=2))
     print(out,flush=True)
    finally:
     if p.poll() is None:
      os.killpg(p.pid,signal.SIGINT)
      try:p.wait(timeout=12)
      except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
     n.destroy_node();rclpy.shutdown();log.close()


if __name__ == "__main__":
    main()
