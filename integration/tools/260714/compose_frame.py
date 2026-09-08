#!/usr/bin/env python3
"""Per-frame composite: [SLAM top-down map + current position] | [SAM RGB + 6D pose overlay].

Run in sam_yolo env (trimesh, numpy, PIL). Reused for single sample and batch.
"""
import os, sys, json, glob, csv, argparse, math
import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFont

OM = "/home/ldh9501/temp_ws/CLI_environment/objectmemory_ws/object_memory"
sys.path.insert(0, os.path.join(OM, "src"))
from adapters.bag_frame_index import load_frame_timestamps

CAD_CACHE = {}
def cad_bbox(cad_path):
    if cad_path not in CAD_CACHE:
        m = trimesh.load(cad_path, process=False)
        b = m.bounds  # [[minx,miny,minz],[maxx,maxy,maxz]] in CAD units (mm)
        xs = [b[0][0], b[1][0]]; ys = [b[0][1], b[1][1]]; zs = [b[0][2], b[1][2]]
        corners = np.array([[x, y, z] for x in xs for y in ys for z in zs], float)
        CAD_CACHE[cad_path] = corners
    return CAD_CACHE[cad_path]

BOX_EDGES = [(0,1),(0,2),(1,3),(2,3),(4,5),(4,6),(5,7),(6,7),(0,4),(1,5),(2,6),(3,7)]
PAL = {}
_colors = [(255,80,80),(90,220,120),(90,160,255),(255,200,60),(210,120,255),(70,220,220),(255,150,70),(180,180,190),(240,120,180),(150,230,90)]

def color_for(name):
    if name not in PAL:
        PAL[name] = _colors[len(PAL) % len(_colors)]
    return PAL[name]

def load_manifest(frame_root):
    cad = {}
    with open(os.path.join(frame_root, "manifest.csv")) as f:
        for row in csv.DictReader(f):
            cad[row["object"]] = row["cad_path"]
    return cad

def project(pts_cam, K):
    z = np.clip(pts_cam[:, 2], 1e-6, None)
    u = K[0,0]*pts_cam[:,0]/z + K[0,2]
    v = K[1,1]*pts_cam[:,1]/z + K[1,2]
    return np.stack([u, v], 1)

def build_sam_panel(frame_dir, cad_map, W=640, H=480):
    rgb = Image.open(os.path.join(frame_dir, "rgb.png")).convert("RGB")
    K = np.array(json.load(open(os.path.join(frame_dir, "camera.json")))["cam_K"]).reshape(3,3)
    d = ImageDraw.Draw(rgb)
    objs = []
    for pem in sorted(glob.glob(os.path.join(frame_dir, "pem_*"))):
        obj = os.path.basename(pem)[4:]
        jp = os.path.join(pem, "sam6d_results", "detection_pem.json")
        if not os.path.isfile(jp):
            continue
        det = json.load(open(jp))
        if isinstance(det, list):
            det = det[0]
        R = np.array(det["R"], float).reshape(3,3); t = np.array(det["t"], float).reshape(3)
        if obj not in cad_map:
            continue
        corners = cad_bbox(cad_map[obj])
        cc = (R @ corners.T).T + t  # camera frame (mm)
        uv = project(cc, K)
        col = color_for(obj)
        for a, b in BOX_EDGES:
            d.line([tuple(uv[a]), tuple(uv[b])], fill=col, width=2)
        cx, cy = uv[:,0].mean(), uv[:,1].mean()
        d.text((cx-20, cy), f"{obj} {det.get('score',0):.2f}", fill=col)
        objs.append(obj)
    return rgb, objs

def load_traj(p):
    ts=[]; xy=[]; yaw=[]
    for l in open(p):
        s=l.split()
        if len(s)<8: continue
        tx,ty,tz=float(s[1]),float(s[2]),float(s[3]); qx,qy,qz,qw=map(float,s[4:8])
        # heading from quaternion: camera forward (+Z) projected to X-Z plane
        # R@[0,0,1]: fz_x = 2(qx qz + qw qy), fz_z = 1-2(qx^2+qy^2)
        fx = 2*(qx*qz+qw*qy); fz = 1-2*(qx*qx+qy*qy)
        ts.append(float(s[0])); xy.append((tx,tz)); yaw.append(math.atan2(fx,fz))
    return np.array(ts), np.array(xy), np.array(yaw)

def load_pcd_xz(p, maxn=8000):
    pts=[]; hdr=True
    for l in open(p):
        if hdr:
            if l.startswith("DATA"): hdr=False
            continue
        s=l.split()
        if len(s)>=3:
            try: pts.append((float(s[0]), float(s[2])))
            except: pass
    if len(pts)>maxn:
        step=len(pts)//maxn; pts=pts[::step]
    return pts

def build_slam_panel(traj_dir, cur_ts, size=900, pad=30, target_h=480, margin=0.6):
    ts, xy, yaw = load_traj(os.path.join(traj_dir, "CameraTrajectory.txt"))
    mp = load_pcd_xz(os.path.join(traj_dir, "orbslam3_map_points.pcd"))
    # view bounds from the CAMERA TRAJECTORY (crop away the far map-point tail)
    xlo,xhi=xy[:,0].min()-margin,xy[:,0].max()+margin
    zlo,zhi=xy[:,1].min()-margin,xy[:,1].max()+margin
    xr, zr = xhi-xlo, zhi-zlo
    # equal aspect; panel HEIGHT fixed to target_h (== RGB height), width follows data
    sc = (target_h - 2*pad) / zr
    Wp = int(xr*sc + 2*pad); Hp = target_h
    def T(x,z): return (pad+(x-xlo)*sc, pad+(zhi-z)*sc)
    img=Image.new("RGB",(Wp,Hp),(20,20,24)); d=ImageDraw.Draw(img)
    gx=math.floor(xlo)
    while gx<=xhi: u,_=T(gx,zlo); d.line([(u,pad),(u,Hp-pad)],fill=(40,40,48)); gx+=1
    gz=math.floor(zlo)
    while gz<=zhi: _,v=T(xlo,gz); d.line([(pad,v),(Wp-pad,v)],fill=(40,40,48)); gz+=1
    r_mp = 1.2
    for p in mp:
        u,v=T(p[0],p[1])
        if -5<=u<=Wp+5 and -5<=v<=Hp+5:
            d.ellipse([u-r_mp,v-r_mp,u+r_mp,v+r_mp],fill=(85,90,108))
    # current index by nearest timestamp
    ci=int(np.argmin(np.abs(ts-cur_ts)))
    # full path faint, traveled path bright
    pts=[T(x,z) for x,z in xy]
    for i in range(1,len(pts)): d.line([pts[i-1],pts[i]],fill=(70,78,105),width=1)
    for i in range(1,ci+1): d.line([pts[i-1],pts[i]],fill=(90,200,255),width=2)
    # current position + heading (scaled to panel size)
    R=max(7, Hp*0.018); AR=max(18, Hp*0.05)
    cu,cv=pts[ci]
    d.ellipse([cu-R,cv-R,cu+R,cv+R],fill=(255,90,90),outline=(255,255,255),width=2)
    hx=cu+AR*math.sin(yaw[ci]); hy=cv-AR*math.cos(yaw[ci])
    d.line([(cu,cv),(hx,hy)],fill=(255,255,120),width=3)
    d.ellipse([T(xy[0,0],xy[0,1])[0]-4,T(xy[0,0],xy[0,1])[1]-4,T(xy[0,0],xy[0,1])[0]+4,T(xy[0,0],xy[0,1])[1]+4],fill=(90,255,120))
    d.text((pad,10),"SLAM_data/105319  ORB-SLAM3 top-down (X-Z, 1m grid)",fill=(230,230,230))
    d.text((pad,26),f"current pos=({xy[ci,0]:.2f},{xy[ci,1]:.2f})m  yaw={math.degrees(yaw[ci]):.0f}deg",fill=(255,200,120))
    return img, ci, xy[ci], ts[ci]

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--frame-dir", required=True)   # .../sam_105314/frame_000356
    ap.add_argument("--pem-root", required=True)     # .../sam_105314 (has manifest.csv)
    ap.add_argument("--sam-bag", required=True)       # converted sam_105314 (for frame ts)
    ap.add_argument("--slam-out", required=True)      # orbslam output dir 105319
    ap.add_argument("--slam-size", type=int, default=900)  # SLAM panel pixel size
    ap.add_argument("--out", required=True)
    a=ap.parse_args()
    cad_map = load_manifest(a.pem_root)
    fidx = int(os.path.basename(a.frame_dir).split("_")[1])
    ts_list = load_frame_timestamps(a.sam_bag, color_topic_hint="/camera/camera/color/image_raw")
    cur_ts = ts_list[fidx]
    sam, objs = build_sam_panel(a.frame_dir, cad_map)
    slam, ci, cpos, sts = build_slam_panel(a.slam_out, cur_ts, size=a.slam_size)
    d=ImageDraw.Draw(sam)
    d.text((6,6),f"SAM_data/105314  frame {fidx}  6D pose overlay  ({len(objs)} obj)",fill=(255,255,0))
    dt=abs(sts-cur_ts)
    W=slam.width+sam.width+30; H=max(slam.height,sam.height)+40
    canvas=Image.new("RGB",(W,H),(12,12,14)); cd=ImageDraw.Draw(canvas)
    canvas.paste(slam,(10,30)); canvas.paste(sam,(slam.width+20,30+(slam.height-sam.height)//2))
    cd.text((10,8),f"frame {fidx}  t={cur_ts:.3f}  (SLAM<->SAM time gap {dt*1000:.0f} ms)",fill=(220,220,230))
    canvas.save(a.out)
    print(f"saved {a.out}  objs={objs}  slam_idx={ci} pos={np.round(cpos,2)} dt={dt*1000:.0f}ms")

if __name__=="__main__":
    main()
