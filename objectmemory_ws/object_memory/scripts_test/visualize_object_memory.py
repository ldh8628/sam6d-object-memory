#!/usr/bin/env python3
"""Visualize object_memory: per-frame composite (left = RGB + 6D pose,
right = 2D BEV map with camera + object landmarks). Pillow only, no numpy.

Left  : input RGB with, per detection, a projected 3D CAD bounding box, object
        coordinate axes, 2D bbox and label.
Right : top-down map on the two highest-variance world axes, showing the camera
        trajectory, current camera pose + heading, and the object landmarks the
        consumer actually needs: only ACTIVE (on screen now) and REMEMBERED (in
        the map but off screen). Tentative/lost/merged/deleted are internal
        bookkeeping and are not drawn.

Single canonical output per run: output/<bag>/viz/frame_XXXXXX.png and
output/<bag>/<bag>.gif (no per-experiment variants).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from PIL import Image, ImageDraw, ImageFont

import bisect
import glob

from adapters.bag_frame_index import load_frame_timestamps
from adapters.cad_extents import load_extents
from adapters.sam6d_pem import load_pem_detections
from adapters.slam_map import default_map_path, load_map_points
from adapters.slam_trajectory import load_slam_trajectory
from core.state_machine import ObjectStatus
from pipeline.object_memory_runner import run_object_memory

# ---- palette (stable per object name) -------------------------------------
_PALETTE = [
    (231, 76, 60), (46, 204, 113), (52, 152, 219), (241, 196, 15),
    (155, 89, 182), (26, 188, 156), (230, 126, 34), (236, 64, 122),
    (149, 165, 166), (99, 110, 250),
]
def _load_font(size, bold=False):
    names = (["DejaVuSans-Bold.ttf", "DejaVuSans.ttf"] if bold
             else ["DejaVuSans.ttf"])
    dirs = ["/snap/gnome-42-2204/247/usr/share/fonts/truetype/dejavu",
            "/usr/share/fonts/truetype/dejavu",
            "/usr/share/fonts/TTF"]
    for dname in dirs:
        for n in names:
            p = os.path.join(dname, n)
            if os.path.isfile(p):
                try:
                    return ImageFont.truetype(p, size)
                except OSError:
                    pass
    return ImageFont.load_default()


_FONT = _load_font(11)                 # small (raw residuals etc.)
_FONT_LABEL = _load_font(14, bold=True)  # object name labels
_FONT_HEAD = _load_font(15, bold=True)   # header / panel titles


def _text_bg(d, xy, text, fill, font=_FONT_LABEL, bg=(0, 0, 0), pad=2):
    """Draw text with a solid background box for readability."""
    x, y = xy
    l, t, r, b = d.textbbox((x, y), text, font=font)
    d.rectangle([l - pad, t - pad, r + pad, b + pad], fill=bg)
    d.text((x, y), text, fill=fill, font=font)


LEFT_W, LEFT_H = 640, 480
RIGHT_W, RIGHT_H = 480, 480
MARGIN = 40
AXIS_LEN_MM = 60.0
BOX_EDGES = [
    (0, 1), (1, 3), (3, 2), (2, 0),  # bottom (z=min)
    (4, 5), (5, 7), (7, 6), (6, 4),  # top    (z=max)
    (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
]


def color_for(name, names_sorted):
    return _PALETTE[names_sorted.index(name) % len(_PALETTE)]


# ---- small linear algebra --------------------------------------------------
def _rot_of(T):
    return ((T[0][0], T[0][1], T[0][2]),
            (T[1][0], T[1][1], T[1][2]),
            (T[2][0], T[2][1], T[2][2]))


def _apply_rot(R, p):
    return (R[0][0]*p[0]+R[0][1]*p[1]+R[0][2]*p[2],
            R[1][0]*p[0]+R[1][1]*p[1]+R[1][2]*p[2],
            R[2][0]*p[0]+R[2][1]*p[1]+R[2][2]*p[2])


def _project(K, P):
    """Pinhole-project a camera-frame point P (mm) -> (u, v) px, or None."""
    fx, fy, cx, cy = K
    if P[2] <= 1e-6:
        return None
    return (fx * P[0] / P[2] + cx, fy * P[1] / P[2] + cy)


def _box_corners(extent):
    (x0, y0, z0), (x1, y1, z1) = extent
    return [(x0, y0, z0), (x1, y0, z0), (x0, y1, z0), (x1, y1, z0),
            (x0, y0, z1), (x1, y0, z1), (x0, y1, z1), (x1, y1, z1)]


# ---- left panel ------------------------------------------------------------
def render_left(rgb_path, K, dets, extents, names_sorted):
    img = Image.open(rgb_path).convert("RGB").resize((LEFT_W, LEFT_H))
    d = ImageDraw.Draw(img)
    for det in dets:
        col = color_for(det.object_name, names_sorted)
        R = _rot_of(det.T_cam_obj)
        t_mm = (det.T_cam_obj[0][3]*1000.0,
                det.T_cam_obj[1][3]*1000.0,
                det.T_cam_obj[2][3]*1000.0)

        # 2D bbox + label
        if det.bbox is not None:
            x, y, w, h = det.bbox
            d.rectangle([x, y, x+w, y+h], outline=col, width=2)
            _text_bg(d, (x, max(0, y-17)), f"{det.object_name} {det.score:.2f}",
                     col, _FONT_LABEL, bg=(0, 0, 0))

        # projected 3D CAD box
        ext = extents.get(det.object_name)
        if ext is not None:
            pts = []
            for c in _box_corners(ext):
                Pc = _apply_rot(R, c)
                Pc = (Pc[0]+t_mm[0], Pc[1]+t_mm[1], Pc[2]+t_mm[2])
                pts.append(_project(K, Pc))
            for a, b in BOX_EDGES:
                if pts[a] and pts[b]:
                    d.line([pts[a], pts[b]], fill=col, width=2)

        # object coordinate axes (X red / Y green / Z blue)
        o = _project(K, t_mm)
        for axis, ac in ((0, (255, 60, 60)), (1, (60, 255, 60)), (2, (80, 120, 255))):
            e = [0.0, 0.0, 0.0]; e[axis] = AXIS_LEN_MM
            Pe = _apply_rot(R, e)
            Pe = _project(K, (Pe[0]+t_mm[0], Pe[1]+t_mm[1], Pe[2]+t_mm[2]))
            if o and Pe:
                d.line([o, Pe], fill=ac, width=2)
    _text_bg(d, (6, 5), "6D pose (SAM-6D)", (255, 255, 255), _FONT_HEAD)
    return img


# ---- right panel (BEV) -----------------------------------------------------
def choose_plane(cam_pts):
    """Return the two world-axis indices with the largest variance (a0<a1)."""
    n = len(cam_pts)
    if n == 0:
        return (0, 2)
    means = [sum(p[i] for p in cam_pts)/n for i in range(3)]
    var = [sum((p[i]-means[i])**2 for p in cam_pts)/n for i in range(3)]
    order = sorted(range(3), key=lambda i: var[i], reverse=True)[:2]
    return tuple(sorted(order))


def compute_bounds(pts2d):
    xs = [p[0] for p in pts2d]; ys = [p[1] for p in pts2d]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if xmax - xmin < 1e-6: xmax += 0.5; xmin -= 0.5
    if ymax - ymin < 1e-6: ymax += 0.5; ymin -= 0.5
    mx = 0.1*(xmax-xmin); my = 0.1*(ymax-ymin)
    return (xmin-mx, xmax+mx, ymin-my, ymax+my)


def make_w2p(bounds, size):
    xmin, xmax, ymin, ymax = bounds
    xr, yr = xmax-xmin, ymax-ymin
    avail = size - 2*MARGIN
    scale = avail / max(xr, yr)
    ox = MARGIN + (avail - xr*scale)/2
    oy = MARGIN + (avail - yr*scale)/2

    def w2p(wx, wy):
        px = ox + (wx - xmin)*scale
        py = size - (oy + (wy - ymin)*scale)   # flip: world +y is up
        return (px, py)
    return w2p, scale


def _pct(sorted_vals, p):
    if not sorted_vals:
        return 0.0
    i = int(p / 100.0 * (len(sorted_vals) - 1))
    return sorted_vals[i]


def build_bev_background(plane, bounds, map_pts, band=(20, 80)):
    """Rasterize the SLAM map cloud into a clean top-down occupancy view.

    Ceiling/floor slabs are removed with a height band on the out-of-plane axis
    (``band`` = keep [lo, hi] percentiles), and the remaining points are binned
    into a density image (denser = brighter) so walls/furniture read clearly."""
    a0, a1 = plane
    hidx = ({0, 1, 2} - {a0, a1}).pop()
    img = Image.new("RGB", (RIGHT_W, RIGHT_H), (20, 22, 27))
    if not map_pts:
        return img
    hs = sorted(p[hidx] for p in map_pts)
    hlo, hhi = _pct(hs, band[0]), _pct(hs, band[1])
    w2p, _ = make_w2p(bounds, RIGHT_H)

    counts = {}
    mx = 0
    for p in map_pts:
        h = p[hidx]
        if h < hlo or h > hhi:
            continue
        x, y = w2p(p[a0], p[a1])
        xi, yi = int(x), int(y)
        if 0 <= xi < RIGHT_W and 0 <= yi < RIGHT_H:
            c = counts.get((xi, yi), 0) + 1
            counts[(xi, yi)] = c
            if c > mx:
                mx = c
    if mx == 0:
        return img
    denom = math.log(mx + 1)
    px = img.load()
    for (xi, yi), c in counts.items():
        t = math.log(c + 1) / denom          # 0..1 density
        v = int(45 + 160 * t)                # grayscale: denser -> brighter
        px[xi, yi] = (v, v, min(255, v + 12))
    return img


_STATUS_TAG = {"active": "A", "lost": "L", "tentative": "T",
               "remembered": "R", "merged": "M", "deleted": "D"}


def _draw_legend(d, registered, names_sorted):
    """Top-right legend: number, colour swatch, name, status for each object.

    Names live here (not on the markers) so nearby markers stay readable.
    Swatch filled = seen this frame, hollow = registered but not seen."""
    if not registered:
        return
    items = sorted(registered, key=lambda o: names_sorted.index(o.object_name))
    lh = 15
    box_w, pad = 190, 5
    box_h = lh * len(items) + 2 * pad
    x0 = RIGHT_W - box_w - 4
    y0 = 4
    d.rectangle([x0, y0, x0 + box_w, y0 + box_h], fill=(0, 0, 0),
                outline=(90, 90, 98))
    for i, ob in enumerate(items):
        col = color_for(ob.object_name, names_sorted)
        n = names_sorted.index(ob.object_name) + 1
        yy = y0 + pad + i * lh
        sx, sy = x0 + 6, yy + 3
        if ob.visible:
            d.ellipse([sx, sy, sx + 9, sy + 9], fill=col, outline=(255, 255, 255))
        elif ob.rejected:
            d.ellipse([sx, sy, sx + 9, sy + 9], outline=(255, 80, 80), width=2)
        else:
            d.ellipse([sx, sy, sx + 9, sy + 9], outline=col, width=2)
        tag = _STATUS_TAG.get(ob.status.value, "?")
        vis = "*" if ob.visible else ("!" if ob.rejected else " ")
        d.text((x0 + 20, yy + 1), f"{n} {ob.object_name}", fill=col, font=_FONT)
        d.text((x0 + box_w - 26, yy + 1), f"{tag}{vis}", fill=(200, 200, 200),
               font=_FONT)


# The consumer view: only these statuses are drawn. active = on screen now,
# remembered = in the map but off screen. Everything else (tentative, lost,
# merged, deleted) is internal lifecycle bookkeeping and is intentionally hidden.
PUBLISHED_STATUSES = (ObjectStatus.active, ObjectStatus.remembered)


def render_right(plane, bounds, base_bg, traj_all, traj_upto, cam_pt, cam_fwd,
                 objects, names_sorted, disp):
    """disp = dict(min_obs). Only active+remembered landmarks are drawn."""
    a0, a1 = plane
    img = base_bg.copy()
    d = ImageDraw.Draw(img)
    w2p, scale = make_w2p(bounds, RIGHT_H)

    def P(w3):
        return w2p(w3[a0], w3[a1])

    # frame + axis labels
    d.rectangle([1, 1, RIGHT_W-2, RIGHT_H-2], outline=(70, 74, 82))
    axis_name = {0: "X", 1: "Y", 2: "Z"}
    _text_bg(d, (6, 5), "2D map (SLAM)", (235, 235, 235), _FONT_HEAD)
    _text_bg(d, (6, 26), "o raw   * fused   x rejected", (170, 170, 170), _FONT)
    d.text((RIGHT_W//2-4, RIGHT_H-16), axis_name[a0], fill=(160, 160, 160), font=_FONT)
    d.text((5, RIGHT_H//2-7), axis_name[a1], fill=(160, 160, 160), font=_FONT)

    # full trajectory (faint) then traveled portion (bright)
    if len(traj_all) > 1:
        d.line([P(p) for p in traj_all], fill=(70, 74, 82), width=1)
    if len(traj_upto) > 1:
        d.line([P(p) for p in traj_upto], fill=(120, 170, 220), width=2)

    # consumer view: draw only active + remembered landmarks (see
    # PUBLISHED_STATUSES). Both legend and markers use the same set so a
    # remembered (off-screen) object still shows as a hollow ring on the map.
    def base_keep(ob):
        return ob.obs >= disp["min_obs"] and ob.status in PUBLISHED_STATUSES

    registered = [ob for ob in objects if base_keep(ob)]
    marker_objs = registered

    def num_of(ob):
        return names_sorted.index(ob.object_name) + 1

    def num_center(cx, cy, s, fill):
        d.text((cx, cy - 1), s, fill=fill, font=_FONT, anchor="mm")

    # markers: colored circles + a small index number inside; NO name text
    # (names live in the legend) so nearby circles stay readable.
    for ob in sorted(marker_objs, key=lambda o: o.visible):  # visible on top
        col = color_for(ob.object_name, names_sorted)
        cx, cy = P(ob.T_map_obj_xyz)
        n = str(num_of(ob))
        if ob.visible:
            if cam_pt is not None:
                cpx, cpy = P(cam_pt)
                d.line([(cpx, cpy), (cx, cy)], fill=(110, 110, 118), width=1)
            if ob.meas_xyz is not None:                  # raw vs fused
                mx, my = P(ob.meas_xyz)
                dim = tuple(int(c * 0.6) for c in col)
                d.line([(mx, my), (cx, cy)], fill=dim, width=1)
                d.ellipse([mx-3, my-3, mx+3, my+3], outline=dim, width=1)
            r = 9
            d.ellipse([cx-r, cy-r, cx+r, cy+r], fill=col, outline=(255, 255, 255), width=2)
            num_center(cx, cy, n, (255, 255, 255))
        elif ob.rejected:
            # detected this frame but the measurement was rejected by the
            # association gate: red-ringed landmark + red x at the rejected
            # raw measurement, so poisoned landmarks are visible at a glance.
            rej = (255, 80, 80)
            if ob.meas_xyz is not None:
                mx, my = P(ob.meas_xyz)
                d.line([(mx-4, my-4), (mx+4, my+4)], fill=rej, width=2)
                d.line([(mx-4, my+4), (mx+4, my-4)], fill=rej, width=2)
                d.line([(mx, my), (cx, cy)], fill=(150, 60, 60), width=1)
            r = 8
            d.ellipse([cx-r, cy-r, cx+r, cy+r], outline=rej, width=3)
            num_center(cx, cy, n, rej)
        else:
            # registered but not seen this frame: full-colour hollow ring.
            r = 8
            d.ellipse([cx-r, cy-r, cx+r, cy+r], outline=col, width=3)
            num_center(cx, cy, n, col)

    # legend (top-right): number, colour, name, status for every registered obj
    _draw_legend(d, registered, names_sorted)

    # current camera + heading arrow (drawn last, on top)
    if cam_pt is not None:
        cpx, cpy = P(cam_pt)
        if cam_fwd is not None:
            fx, fy = cam_fwd[a0], cam_fwd[a1]
            nrm = math.hypot(fx, fy)
            if nrm > 1e-9:
                L = 30.0
                ex = cpx + (fx/nrm)*L
                ey = cpy - (fy/nrm)*L   # flip y to screen
                d.line([(cpx, cpy), (ex, ey)], fill=(255, 255, 0), width=3)
                d.ellipse([ex-3, ey-3, ex+3, ey+3], fill=(255, 255, 0))
        d.ellipse([cpx-6, cpy-6, cpx+6, cpy+6], fill=(255, 255, 255),
                  outline=(0, 0, 0), width=2)
        _text_bg(d, (cpx+8, cpy+4), "cam", (255, 255, 255), _FONT, bg=(0, 0, 0))
    return img


class _Obj:  # lightweight snapshot holder with pre-extracted xyz
    __slots__ = ("object_name", "status", "visible", "rejected", "obs",
                 "T_map_obj_xyz", "meas_xyz", "residual_t", "residual_deg")

    def __init__(self, s):
        self.object_name = s.object_name
        self.status = s.status
        self.visible = s.visible
        self.rejected = getattr(s, "rejected_this_frame", False)
        self.obs = s.observation_count
        self.T_map_obj_xyz = (s.T_map_obj[0][3], s.T_map_obj[1][3], s.T_map_obj[2][3])
        m = s.T_map_obj_meas
        self.meas_xyz = (m[0][3], m[1][3], m[2][3]) if m else None
        self.residual_t = s.residual_t
        self.residual_deg = s.residual_deg

    def hidden(self):
        """Copy of this object marked not-visible (for carry-forward frames)."""
        o = _Obj.__new__(_Obj)
        o.object_name = self.object_name
        o.status = self.status
        o.visible = False
        o.rejected = False
        o.obs = self.obs
        o.T_map_obj_xyz = self.T_map_obj_xyz
        o.meas_xyz = None
        o.residual_t = None
        o.residual_deg = None
        return o


def compose(left, right, header):
    W, H = LEFT_W + RIGHT_W, LEFT_H + 26
    canvas = Image.new("RGB", (W, H), (0, 0, 0))
    canvas.paste(left, (0, 26))
    canvas.paste(right, (LEFT_W, 26))
    ImageDraw.Draw(canvas).text((6, 5), header, fill=(255, 255, 255), font=_FONT_HEAD)
    return canvas


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="object_memory visualizer")
    ap.add_argument("--slam-traj", required=True)
    ap.add_argument("--pem-dir", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--frames-dir", default=None,
                    help="dir with frame_XXXXXX/{rgb.png,camera.json}; "
                         "default = <bag_root>/input derived from --pem-dir")
    ap.add_argument("--cad-root", default=None,
                    help="CAD root; default = <repo>/sam6d_ws/data/cad")
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--source-slam-id", default="orbslam3_noimu")
    ap.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--time-tolerance", type=float, default=0.05)
    ap.add_argument("--assoc-trans-gate", type=float, default=0.20,
                    help="multi-instance association radius (m)")
    ap.add_argument("--assoc-rot-gate", type=float, default=-1.0,
                    help="rotation gate (deg); <0 disables it")
    ap.add_argument("--legacy-confidence", action="store_true",
                    help="use the pre-Story-5 hit-counter existence update "
                         "instead of quality-weighted confidence (default)")
    ap.add_argument("--no-gif", action="store_true")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--slam-map", default=None,
                    help="SLAM map pcd for the BEV background; "
                         "default = dense map next to --slam-traj")
    ap.add_argument("--no-map", action="store_true",
                    help="disable the SLAM-map background")
    ap.add_argument("--map-max-points", type=int, default=200000)
    ap.add_argument("--map-source", choices=["orb", "rtabmap"], default="orb",
                    help="SLAM map cloud source for the BEV background")
    ap.add_argument("--map-band", type=float, nargs=2, default=(20.0, 80.0),
                    metavar=("LO", "HI"),
                    help="keep [LO,HI] height percentiles (drops ceiling/floor)")
    ap.add_argument("--plane", choices=["auto", "xy", "xz", "yz"], default="auto",
                    help="BEV map plane; 'auto' = two highest-variance axes")
    ap.add_argument("--min-obs", type=int, default=1,
                    help="hide objects with fewer than N accepted observations")
    ap.add_argument("--x-npy", default=None,
                    help="4x4 T_camSLAM_camSAM extrinsic (.npy) for a two-camera rig: the "
                         "trajectory is the SLAM camera's, the detections are the SAM "
                         "camera's. Applied to the pose stream.")
    args = ap.parse_args(argv)

    here = os.path.dirname(__file__)
    repo = os.path.abspath(os.path.join(here, "..", "..", ".."))
    output_root = os.path.abspath(os.path.join(here, "..", "..", "output"))

    real_pem = os.path.realpath(args.pem_dir)
    bag_root = os.path.dirname(os.path.dirname(real_pem))  # <bag>/output/pem -> <bag>
    # bag name: the layout folder; if --bag ends in 'bag', use its parent.
    bn = os.path.basename(os.path.normpath(args.bag))
    bag_name = os.path.basename(bag_root) if bn == "bag" else bn
    frames_dir = args.frames_dir or os.path.join(bag_root, "input")
    cad_root = args.cad_root or os.path.join(repo, "sam6d_ws", "data", "cad")
    # single canonical output dir per bag (no per-experiment variants).
    out_dir = args.out_dir or os.path.join(output_root, bag_name, "viz")
    gif_path = os.path.join(output_root, bag_name, f"{bag_name}.gif")
    os.makedirs(out_dir, exist_ok=True)

    poses = load_slam_trajectory(args.slam_traj, source_slam_id=args.source_slam_id)
    if args.x_npy:
        # Two-camera rig: the trajectory belongs to the SLAM camera, the detections to
        # the SAM camera. Move the POSE stream onto the SAM camera (T_map_camSAM =
        # T_map_camSLAM * X) so placement AND the visibility model use the camera that
        # actually made the detection -- see run_object_memory_extrinsic.py.
        import dataclasses as _dc
        import numpy as _np
        from core.transforms import compose_transform as _cx
        _X = tuple(tuple(float(v) for v in row) for row in _np.load(args.x_npy))
        poses = [_dc.replace(p, T_map_cam=_cx(p.T_map_cam, _X)) for p in poses]
        print(f"extrinsic applied to poses from {args.x_npy}")
    timestamps = load_frame_timestamps(args.bag, color_topic_hint=args.color_topic)
    detections = load_pem_detections(args.pem_dir, timestamps=timestamps)
    # multi-instance production config (matches run_object_memory.py CLI
    # defaults): map-space radius 0.20 m, rotation gate off (PEM rot is noisy).
    result = run_object_memory(poses, detections, timestamps,
                               time_tolerance=args.time_tolerance,
                               assoc_trans_gate_m=args.assoc_trans_gate,
                               assoc_rot_gate_deg=args.assoc_rot_gate,
                               quality_weighting=not args.legacy_confidence)

    names_sorted = sorted({d.object_name
                           for ds in detections.values() for d in ds})
    extents = load_extents(cad_root, names_sorted,
                           cache_path=os.path.join(output_root, "_cad_extents.json"))
    missing = [n for n in names_sorted if n not in extents]

    # ---- SLAM map cloud (BEV background) ---------------------------------
    map_pts = []
    map_path = ""
    if not args.no_map:
        if args.slam_map:
            map_path = args.slam_map
        elif args.map_source == "rtabmap":
            map_path = os.path.join(repo, "slam_comparison", "output", bag_name,
                                    "rtabmap", "rtabmap_3d_map_cloud.ply")
        else:  # orb: dense map next to the trajectory, else in slam_comparison
            map_path = default_map_path(args.slam_traj)
            if not (map_path and os.path.isfile(map_path)):
                slam_dir = os.path.join(repo, "slam_comparison", "output",
                                        bag_name, args.source_slam_id)
                map_path = default_map_path(
                    os.path.join(slam_dir, "CameraTrajectory.txt"))
        if map_path and os.path.isfile(map_path):
            map_pts = load_map_points(map_path, max_points=args.map_max_points)

    # ---- global BEV plane + bounds (stable across frames) ----------------
    cam_pts = [(p.T_map_cam[0][3], p.T_map_cam[1][3], p.T_map_cam[2][3])
               for p in poses]
    obj_pts = [(lm.T_map_obj[0][3], lm.T_map_obj[1][3], lm.T_map_obj[2][3])
               for lm in result.store.all_landmarks()]
    plane = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}.get(
        args.plane, choose_plane(cam_pts))
    a0, a1 = plane
    bound_pts = [(p[a0], p[a1]) for p in (cam_pts + obj_pts)]
    if map_pts:  # frame the map extent robustly (2..98 pct, ignore outliers)
        xs = sorted(p[a0] for p in map_pts)
        ys = sorted(p[a1] for p in map_pts)
        bound_pts += [(_pct(xs, 2), _pct(ys, 2)), (_pct(xs, 98), _pct(ys, 98))]
    bounds = compute_bounds(bound_pts)
    traj_all = cam_pts
    base_bg = build_bev_background(plane, bounds, map_pts,
                                  band=tuple(args.map_band))
    disp = {"min_obs": args.min_obs}

    # ---- camera pose lookup for every frame (incl. non-detection) --------
    pstamps = [p.stamp for p in poses]

    def nearest_cam(ts, tol=0.1):
        if ts is None or not pstamps:
            return None
        i = bisect.bisect_left(pstamps, ts)
        best, best_dt = None, tol
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(pstamps):
                dt = abs(pstamps[j] - ts)
                if dt <= best_dt:
                    best_dt, best = dt, poses[j]
        return best

    fr_by_idx = {fr.frame_idx: fr for fr in result.frame_results}

    # every sampled input frame (so the GIF is continuous, not just detections)
    input_idx = sorted(
        int(os.path.basename(p).split("_")[1])
        for p in glob.glob(os.path.join(frames_dir, "frame_*"))
        if os.path.isdir(p) and os.path.basename(p).split("_")[1].isdigit()
    )
    if args.max_frames:
        input_idx = input_idx[:args.max_frames]

    saved = []
    last_objs = []          # carry-forward memory snapshot
    n_with_det = 0
    for idx in input_idx:
        rgb_path = os.path.join(frames_dir, f"frame_{idx:06d}", "rgb.png")
        cam_json = os.path.join(frames_dir, f"frame_{idx:06d}", "camera.json")
        if not (os.path.isfile(rgb_path) and os.path.isfile(cam_json)):
            continue
        with open(cam_json) as f:
            k = json.load(f)["cam_K"]
        K = (k[0], k[4], k[2], k[5])

        ts = timestamps[idx] if 0 <= idx < len(timestamps) else None
        fr = fr_by_idx.get(idx)
        dets = detections.get(idx, []) if fr is not None else []

        # object snapshot + camera pose for this frame
        if fr is not None:
            objs = [_Obj(s) for s in fr.objects]
            last_objs = objs
            n_with_det += 1
            T = fr.camera_T_map_cam
        else:
            objs = [o.hidden() for o in last_objs]   # persist last-known state
            T = None
        cam_pt = cam_fwd = None
        pose = nearest_cam(ts)
        if pose is not None:
            Tc = pose.T_map_cam
            cam_pt = (Tc[0][3], Tc[1][3], Tc[2][3])
            cam_fwd = (Tc[0][2], Tc[1][2], Tc[2][2])  # world dir of optical +Z

        left = render_left(rgb_path, K, dets, extents, names_sorted)
        traj_upto = [c for c, p in zip(cam_pts, poses)
                     if ts is not None and p.stamp <= ts]
        right = render_right(plane, bounds, base_bg, traj_all, traj_upto,
                             cam_pt, cam_fwd, objs, names_sorted, disp)

        # header reports the published view (what is actually drawn), not the
        # internal snapshot which also holds tentative/lost bookkeeping.
        active = sum(1 for o in objs if o.status == ObjectStatus.active)
        remembered = sum(1 for o in objs
                         if o.status == ObjectStatus.remembered)
        vis = sum(1 for o in objs if o.visible)
        det_tag = f"det={len(dets)}" if fr is not None else "det=0 (no detection)"
        slam = "SLAM ok" if pose is not None else "no SLAM"
        header = (f"{bag_name}  frame {idx:06d}  |  {slam}  |  {det_tag}  |  "
                  f"active={active} remembered={remembered} visible={vis}")
        out = compose(left, right, header)
        out_path = os.path.join(out_dir, f"frame_{idx:06d}.png")
        out.save(out_path)
        saved.append(out_path)

    print(f"bag: {bag_name}")
    print(f"map: {os.path.basename(map_path) if map_pts else '(none)'} "
          f"({len(map_pts)} pts)")
    print(f"plane: axes {a0},{a1}  frames rendered: {len(saved)} "
          f"(with detections: {n_with_det})")
    print(f"objects: {names_sorted}")
    if missing:
        print(f"no CAD (axes only, no 3D box): {missing}")
    print(f"out dir: {out_dir}")

    if saved and not args.no_gif:
        frames_img = [Image.open(p) for p in saved]
        frames_img[0].save(gif_path, save_all=True, append_images=frames_img[1:],
                           duration=300, loop=0)
        print(f"gif: {gif_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
