"""torch RNG 까지 고정하면 결정적인가 — 흔들림의 출처가 coarse matching 의 무작위 가설인가."""
import json, sys, os, numpy as np, cv2, torch, hashlib
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import pem_probe as PP
from segment_anything.utils.amg import mask_to_rle_pytorch
os.makedirs(PP.TMP, exist_ok=True)
PR = Path(sys.argv[1])
dirs = sorted(p for p in PR.iterdir() if (p/"meta.json").is_file())
names = sorted({o for p in dirs for o in json.load(open(p/"meta.json"))["objects"]})
ric, cfg, net, tem = PP.load_pem("cuda:0", names)
def geo(A,B): return float(np.degrees(np.arccos(np.clip((np.trace(A.T@B)-1)/2,-1,1))))
out=[]
for p in dirs:
    m=json.load(open(p/"meta.json")); K=np.array(m["cam_K"],float).reshape(3,3)
    json.dump({"cam_K":K.flatten().tolist(),"depth_scale":1.0},open(f"{PP.TMP}/camera.json","w"))
    lab=cv2.imread(str(p/"mask.png"),0)
    for oi,o in enumerate(m["objects"]):
        mask=(lab==oi+1)
        if mask.sum()<32: continue
        h,w=mask.shape
        rle=mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
        ys,xs=np.where(mask)
        json.dump([{"scene_id":0,"image_id":0,"category_id":1,
                    "bbox":[int(xs.min()),int(ys.min()),int(xs.max()-xs.min()),int(ys.max()-ys.min())],
                    "score":1.0,"segmentation":{"size":[h,w],"counts":[int(c) for c in rle["counts"]]}}],
                  open(f"{PP.TMP}/det.json","w"))
        np.random.seed(7)
        try:
            inp=ric.get_test_data(str(p/"rgb.png"),str(p/"depth.png"),f"{PP.TMP}/camera.json",
                                  o,f"{PP.TMP}/det.json",0.2,cfg.test_dataset)[0]
        except Exception: continue
        tp,tf=tem[o]
        def fwd(seed=None):
            if seed is not None:
                torch.manual_seed(seed); torch.cuda.manual_seed_all(seed)
            x={k:(v.clone() if torch.is_tensor(v) else v) for k,v in inp.items()}
            n=x["pts"].size(0); x["dense_po"]=tp.repeat(n,1,1); x["dense_fo"]=tf.repeat(n,1,1)
            with torch.inference_mode(): r=net(x)
            return r["pred_R"].detach().cpu().numpy()[0]
        fixed=[fwd(2024) for _ in range(3)]
        free=[fwd(None) for _ in range(3)]
        out.append((o, max(geo(fixed[0],fixed[i]) for i in (1,2)),
                       max(geo(free[0],free[i]) for i in (1,2))))
import collections
print(f"\n{'객체':>22} {'n':>4} {'torch씨앗 고정':>14} {'씨앗 자유':>12}")
g=collections.defaultdict(list)
for o,a,b in out: g[o].append((a,b))
for o in sorted(g):
    v=np.array(g[o])
    print(f"{o:>22} {len(v):>4} {np.max(v[:,0]):>14.3f} {np.max(v[:,1]):>12.1f}   (최대값)")
v=np.array([[a,b] for _,a,b in out])
print(f"{'전체':>22} {len(v):>4} {np.max(v[:,0]):>14.3f} {np.max(v[:,1]):>12.1f}")
