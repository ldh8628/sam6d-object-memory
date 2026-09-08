import pyrealsense2 as rs
import sys,time,json
ctx=rs.context();dev=ctx.load_device(sys.argv[1]);dev.set_real_time(False)
rows=[]; sensors=dev.query_sensors()
for s in sensors:s.open(s.get_stream_profiles())
for s in sensors:s.start(lambda f:rows.append([str(f.get_profile().stream_type()),f.get_frame_number()]))
start=time.monotonic()
while time.monotonic()-start<5:
    if rows and dev.current_status()==rs.playback_status.stopped:break
    time.sleep(.01)
print('status',dev.current_status(),'pos',dev.get_position(),flush=True)
for s in sensors:s.stop();s.close()
print(json.dumps(dict(count=len(rows),first=rows[:8],last=rows[-8:]),indent=2))
