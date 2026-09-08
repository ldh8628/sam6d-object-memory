import pyrealsense2 as rs
import sys,time,json
p=rs.pipeline();cfg=rs.config();cfg.enable_device_from_file(sys.argv[1],False)
cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
profile=cfg.resolve(p);dev=profile.get_device().as_playback();dev.set_real_time(False)
rows=[]
def frame(f):
    fs=f.as_frameset();c=fs.get_color_frame();d=fs.get_depth_frame()
    rows.append([c.get_frame_number() if c else None,d.get_frame_number() if d else None,
                 str(c.get_frame_timestamp_domain()) if c else None])
active=p.start(cfg,frame)
dev=active.get_device().as_playback();dev.set_real_time(False)
print("duration",dev.get_duration(),flush=True)
start=time.monotonic()
while time.monotonic()-start<10:
    if rows and dev.current_status()==rs.playback_status.stopped:break
    time.sleep(.01)
print("status",dev.current_status(),"position",dev.get_position(),flush=True)
p.stop()
print(json.dumps(dict(count=len(rows),first=rows[:10],last=rows[-5:],incomplete=sum(not all(r[:2]) for r in rows)),indent=2))
