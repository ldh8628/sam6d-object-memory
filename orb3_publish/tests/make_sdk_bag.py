#!/usr/bin/env python3
"""Generate a RealSense software-device recording to exercise native SDK capture.
The content is replayed TUM RGB-D, not a recording of the user's camera.
"""
import argparse
import gc
from pathlib import Path
import sys
import time
import cv2
import pyrealsense2 as rs

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from run_orb3_publish import load_settings


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('dataset',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--frames',type=int,default=90);ap.add_argument('--gap-at',type=int,default=0)
    ap.add_argument('--start-number',type=int,default=1,help='SDK counters can start at zero; useful for loss-detection regression')
    args=ap.parse_args()
    if args.start_number<0 or args.frames<1 or args.gap_at<0:raise ValueError('Invalid frame count or counter')
    if args.output.exists(): raise ValueError('Output already exists')
    cfg=load_settings(ROOT/'orb3_publish/config/TUM1.yaml')
    device=rs.software_device()
    device.register_info(rs.camera_info.name,'ORB3 validation software camera')
    device.register_info(rs.camera_info.serial_number,'SOFTWARE_TEST_ONLY')
    device.register_info(rs.camera_info.product_line,'D400')
    device.create_matcher(rs.matchers.default)
    intr=rs.intrinsics();intr.width=640;intr.height=480
    intr.fx=cfg['Camera1.fx'];intr.fy=cfg['Camera1.fy'];intr.ppx=cfg['Camera1.cx'];intr.ppy=cfg['Camera1.cy']
    # Both test streams are already registered to the color camera. Zero distortion
    # here makes SDK alignment exactly identity; ORB still uses the TUM settings.
    intr.model=rs.distortion.none;intr.coeffs=[0]*5
    profiles=[]; sensors=[]
    for index,(kind,bpp,fmt,name) in enumerate(((rs.stream.depth,2,rs.format.z16,'Depth'),(rs.stream.color,3,rs.format.bgr8,'Color'))):
        sensor=device.add_sensor(name)
        sensor.add_read_only_option(rs.option.depth_units,0.0002)
        vs=rs.video_stream();vs.type=kind;vs.index=0;vs.uid=index+1;vs.width=640;vs.height=480
        vs.fps=30;vs.bpp=bpp;vs.fmt=fmt;vs.intrinsics=intr
        profile=sensor.add_video_stream(vs).as_video_stream_profile()
        profiles.append(profile);sensors.append(sensor)
    ext=rs.extrinsics();ext.rotation=[1,0,0,0,1,0,0,0,1];ext.translation=[0,0,0]
    profiles[0].register_extrinsics_to(profiles[1],ext)
    recorder=rs.recorder(str(args.output),device,False)
    for sensor,profile in zip(sensors,profiles):
        sensor.open(profile);sensor.start(lambda frame: None)
    rows=(args.dataset/'associations.txt').read_text().splitlines()[:args.frames]
    # Decode the first pair before starting the acquisition clock. Otherwise
    # first-use decoder setup creates an artificial catch-up burst in the bag.
    first=rows[0].split()
    first_rgb=cv2.imread(str(args.dataset/first[1]))
    first_depth=cv2.imread(str(args.dataset/first[3]),cv2.IMREAD_UNCHANGED)
    start=time.monotonic()
    for index,line in enumerate(rows,1):
        stamp,rgb_path,_,depth_path=line.split()
        if index==1:rgb,depth=first_rgb,first_depth
        else:
            rgb=cv2.imread(str(args.dataset/rgb_path));depth=cv2.imread(str(args.dataset/depth_path),cv2.IMREAD_UNCHANGED)
        time.sleep(max(0,start+(index-1)/30-time.monotonic()))
        number=args.start_number+index-1+(2 if args.gap_at and index>=args.gap_at else 0)
        for sensor,profile,pixels,bpp in zip(sensors,profiles,(depth,rgb),(2,3)):
            frame=rs.software_video_frame();frame.pixels=pixels;frame.bpp=bpp;frame.stride=640*bpp
            frame.timestamp=float(stamp)*1000;frame.domain=rs.timestamp_domain.global_time
            frame.frame_number=number;frame.profile=profile;frame.depth_units=.0002
            sensor.on_video_frame(frame)
    for sensor in sensors: sensor.stop();sensor.close()
    recorder.pause();del recorder;gc.collect()
    print(args.output,'frames per stream:',len(rows),'injected counter gap at:',args.gap_at)

if __name__=='__main__':main()
