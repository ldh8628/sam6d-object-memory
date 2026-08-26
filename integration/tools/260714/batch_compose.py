#!/usr/bin/env python3
"""Batch: one composite PNG per stride-5 frame of SAM_data/105314.
[SLAM_data/105319 top-down map + current pos] | [SAM RGB + 6D pose overlay].
SLAM base (map points + faint full path) is precomputed once; per-frame only the
traveled path + current marker are redrawn. RGB comes from the ISM frame_dir when
the frame has detections, else it is decoded from the converted SAM bag.
Run in sam_yolo env.
"""
import os, sys, json, glob, csv, math, struct, sqlite3
import numpy as np
import trimesh
from PIL import Image, ImageDraw

FRAME_H = 480
SAM_BAG = "/home/ldh9501/temp_ws/CLI_environment/data_slam_converted/260714/105319__unlabeled/SAM"
PEM_ROOT = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws/outputs/pem_inputs/sam_105314"
SLAM_OUT = "/home/ldh9501/temp_ws/CLI_environment/orbslam_ws/output/260714/slam_105319"
OUT_DIR = "/home/ldh9501/temp_ws/CLI_environment/output_slam/260714_frame_data/105314_frames"
os.makedirs(OUT_DIR, exist_ok=True)

# ---------- CAD + pose overlay ----------
CAD_CACHE = {}
def cad_corners(cad):
    if cad not in CAD_CACHE:
        b = trimesh.load(cad, process=False).bounds
        xs=[b[0][0],b[1][0]]; ys=[b[0][1],b[1][1]]; zs=[b[0][2],b[1][2]]
        CAD_CACHE[cad]=np.array([[x,y,z] for x in xs for y in ys for z in zs],float)
    return CAD_CACHE[cad]
EDGES=[(0,1),(0,2),(1,3),(2,3),(4,5),(4,6),(5,7),(6,7),(0,4),(1,5),(2,6),(3,7)]
_cols=[(255,80,80),(90,220,120),(90,160,255),(255,200,60),(210,120,255),(70,220,220),(255,150,70),(180,180,190),(240,120,180),(150,230,90)]
PAL={}
def color_for(n):
    if n not in PAL: PAL[n]=_cols[len(PAL)%len(_cols)]
    return PAL[n]

def load_manifest():
    cad={}
    mp=os.path.join(PEM_ROOT,"manifest.csv")
    if os.path.isfile(mp):
        for r in csv.DictReader(open(mp)): cad[r["object"]]=r["cad_path"]
    return cad
CAD_MAP=load_manifest()

def overlay_poses(rgb, frame_dir):
    K=np.array(json.load(open(os.path.join(frame_dir,"camera.json")))["cam_K"]).reshape(3,3)
    d=ImageDraw.Draw(rgb); objs=[]
    for pem in sorted(glob.glob(os.path.join(frame_dir,"pem_*"))):
        if not os.path.isdir(pem): continue
        obj=os.path.basename(pem)[4:]
        jp=os.path.join(pem,"sam6d_results","detection_pem.json")
        if not os.path.isfile(jp) or obj not in CAD_MAP: continue
        det=json.load(open(jp)); det=det[0] if isinstance(det,list) else det
        R=np.array(det["R"],float).reshape(3,3); t=np.array(det["t"],float).reshape(3)
        cc=(R@cad_corners(CAD_MAP[obj]).T).T+t
        z=np.clip(cc[:,2],1e-6,None)
        uv=np.stack([K[0,0]*cc[:,0]/z+K[0,2], K[1,1]*cc[:,1]/z+K[1,2]],1)
        col=color_for(obj)
        for a,b in EDGES: d.line([tuple(uv[a]),tuple(uv[b])],fill=col,width=2)
        d.text((uv[:,0].mean()-18,uv[:,1].mean()),f"{obj} {det.get('score',0):.2f}",fill=col)
        objs.append(obj)
    return objs

# ---------- SAM bag RGB decode ----------
def _img_bytes(blob):
    off=4; off+=8
    fl=struct.unpack_from('<I',blob,off)[0]; off+=4; off+=fl; off=(off+3)&~3
    h,w=struct.unpack_from('<II',blob,off); off+=8
    el=struct.unpack_from('<I',blob,off)[0]; off+=4; off+=el
    off+=1; off=(off+3)&~3; off+=4
    dl=struct.unpack_from('<I',blob,off)[0]; off+=4
    return w,h,blob[off:off+dl]
def load_bag_rgb():
    db=glob.glob(os.path.join(SAM_BAG,"*.db3"))[0]
    c=sqlite3.connect(db); cur=c.cursor()
    cur.execute("SELECT id FROM topics WHERE name='/camera/camera/color/image_raw'")
    tid=cur.fetchone()[0]
    cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(tid,))
    rows=cur.fetchall(); c.close()
    ts=[r[0]/1e9 for r in rows]
    return ts,[r[1] for r in rows]
def rgb_at(blobs, idx):
    w,h,data=_img_bytes(blobs[idx])
    b,g,r=Image.frombytes('RGB',(w,h),bytes(data)).split()
    return Image.merge('RGB',(r,g,b))

# ---------- SLAM base (precompute once) ----------
def load_traj():
    ts=[];xy=[];yaw=[]
    for l in open(os.path.join(SLAM_OUT,"CameraTrajectory.txt")):
        s=l.split()
        if len(s)<8: continue
        tx,tz=float(s[1]),float(s[3]); qx,qy,qz,qw=map(float,s[4:8])
        fx=2*(qx*qz+qw*qy); fz=1-2*(qx*qx+qy*qy)
        ts.append(float(s[0]));xy.append((tx,tz));yaw.append(math.atan2(fx,fz))
    return np.array(ts),np.array(xy),np.array(yaw)
def load_pcd():
    p=os.path.join(SLAM_OUT,"orbslam3_map_points.pcd"); pts=[];hdr=True
    for l in open(p):
        if hdr:
            if l.startswith("DATA"): hdr=False
            continue
        s=l.split()
        if len(s)>=3:
            try: pts.append((float(s[0]),float(s[2])))
            except: pass
    return pts

TS,XY,YAW=load_traj(); MP=load_pcd()
pad=30; margin=0.6
xlo,xhi=XY[:,0].min()-margin,XY[:,0].max()+margin
zlo,zhi=XY[:,1].min()-margin,XY[:,1].max()+margin
SC=(FRAME_H-2*pad)/(zhi-zlo)
WP=int((xhi-xlo)*SC+2*pad); HP=FRAME_H
def T(x,z): return (pad+(x-xlo)*SC, pad+(zhi-z)*SC)
BASE=Image.new("RGB",(WP,HP),(20,20,24)); bd=ImageDraw.Draw(BASE)
gx=math.floor(xlo)
while gx<=xhi: u,_=T(gx,zlo); bd.line([(u,pad),(u,HP-pad)],fill=(40,40,48)); gx+=1
gz=math.floor(zlo)
while gz<=zhi: _,v=T(xlo,gz); bd.line([(pad,v),(WP-pad,v)],fill=(40,40,48)); gz+=1
for p in MP:
    u,v=T(p[0],p[1])
    if -5<=u<=WP+5 and -5<=v<=HP+5: bd.ellipse([u-1.2,v-1.2,u+1.2,v+1.2],fill=(85,90,108))
PTS=[T(x,z) for x,z in XY]
for i in range(1,len(PTS)): bd.line([PTS[i-1],PTS[i]],fill=(70,78,105),width=1)
SU,SV=PTS[0]
bd.ellipse([SU-4,SV-4,SU+4,SV+4],fill=(90,255,120))

SYNC_TOL = 0.12  # s; beyond this the SAM frame has no synchronized SLAM pose
def slam_panel(cur_ts):
    ci=int(np.argmin(np.abs(TS-cur_ts)))
    gap=abs(TS[ci]-cur_ts)
    img=BASE.copy(); d=ImageDraw.Draw(img)
    d.text((pad,8),"SLAM_data/105319 ORB-SLAM3 top-down (X-Z,1m grid)",fill=(230,230,230))
    if gap>SYNC_TOL:
        # SAM frame outside SLAM's recording window (SLAM starts ~4.4s later) -> no pose
        d.text((pad,24),f"no synchronized SLAM pose (nearest {gap*1000:.0f} ms away)",fill=(255,120,120))
        return img,ci,gap
    for i in range(1,ci+1): d.line([PTS[i-1],PTS[i]],fill=(90,200,255),width=2)
    cu,cv=PTS[ci]; Rr=max(7,HP*0.018); AR=max(18,HP*0.05)
    d.ellipse([cu-Rr,cv-Rr,cu+Rr,cv+Rr],fill=(255,90,90),outline=(255,255,255),width=2)
    d.line([(cu,cv),(cu+AR*math.sin(YAW[ci]),cv-AR*math.cos(YAW[ci]))],fill=(255,255,120),width=3)
    d.text((pad,24),f"pos=({XY[ci,0]:.2f},{XY[ci,1]:.2f})m yaw={math.degrees(YAW[ci]):.0f}deg",fill=(255,200,120))
    return img,ci,gap

# ---------- main loop ----------
def main():
    bag_ts, blobs = load_bag_rgb()
    n = len(blobs)
    print(f"{n} frames to compose")
    for idx in range(n):
        cur_ts = bag_ts[idx]
        fdir = os.path.join(PEM_ROOT, f"frame_{idx:06d}")
        if os.path.isdir(fdir) and os.path.isfile(os.path.join(fdir,"rgb.png")):
            rgb = Image.open(os.path.join(fdir,"rgb.png")).convert("RGB")
            objs = overlay_poses(rgb, fdir)
        else:
            rgb = rgb_at(blobs, idx); objs = []
        dr=ImageDraw.Draw(rgb)
        dr.text((6,6),f"SAM_data/105314 frame {idx} 6D pose ({len(objs)} obj)",fill=(255,255,0))
        slam,ci,gap = slam_panel(cur_ts)
        dt=gap*1000
        W=slam.width+rgb.width+30; H=max(slam.height,rgb.height)+40
        canvas=Image.new("RGB",(W,H),(12,12,14)); cd=ImageDraw.Draw(canvas)
        canvas.paste(slam,(10,30)); canvas.paste(rgb,(slam.width+20,30))
        cd.text((10,8),f"frame {idx}  t={cur_ts:.3f}  SLAM<->SAM gap {dt:.0f}ms  objs={objs}",fill=(220,220,230))
        canvas.save(os.path.join(OUT_DIR,f"frame_{idx:06d}.png"))
        if idx % 50 == 0: print(f"  composed {idx}/{n}")
    print("BATCH COMPOSE DONE ->", OUT_DIR)

if __name__=="__main__":
    main()
