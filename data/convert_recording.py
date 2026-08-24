#!/usr/bin/env python3
"""
Convert a RealSense-recorder rosbag (/device_0/... topics, UNALIGNED depth) into a
standard ROS2 bag that ORB-SLAM3 rgbd_node / SAM-6D can consume:
  /camera/color/image_raw                 (bgr8)
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw (16UC1, mm, aligned to color frame)
  /camera/aligned_depth_to_color/camera_info

Depth is aligned to the color frame using the bag's OWN intrinsics + the
depth->color extrinsic stored in tf/ref_0 (convention C: P_color = R*P_depth + t),
which was validated against the raw data.

Timestamps come from rgbd_timestamp_associations.json (host unix-epoch ns).
Run inside conda env: sam6d_ros_humble or orbslam3.

Usage:
  python convert_recording.py <recording_dir> <out_bag_dir> [--stride N] [--max M]
"""
import sqlite3, json, struct, sys, os, argparse
import numpy as np


def parse_camera_info_str(s):
    d = {}
    for kv in s.split(';'):
        if '=' in kv:
            k, v = kv.split('=', 1)
            d[k.strip()] = v.strip()
    return d


def parse_tf_ref(s):
    # rotation=r0..r8;translation=t0,t1,t2  (row-major R = rs2 depth->color)
    rot = None; trans = None
    for kv in s.split(';'):
        if kv.startswith('rotation='):
            rot = [float(x) for x in kv[len('rotation='):].split(',')]
        elif kv.startswith('translation='):
            trans = [float(x) for x in kv[len('translation='):].split(',')]
    return np.array(rot, dtype=np.float64).reshape(3, 3), np.array(trans, dtype=np.float64)


def read_image_msg(blob):
    off = 4; sec, nsec = struct.unpack_from('<iI', blob, off); off += 8
    flen = struct.unpack_from('<I', blob, off)[0]; off += 4; off += flen; off = (off + 3) & ~3
    h, w = struct.unpack_from('<II', blob, off); off += 8
    elen = struct.unpack_from('<I', blob, off)[0]; off += 4
    enc = blob[off:off + elen - 1].decode(); off += elen
    off += 1; off = (off + 3) & ~3
    step = struct.unpack_from('<I', blob, off)[0]; off += 4
    dlen = struct.unpack_from('<I', blob, off)[0]; off += 4
    return w, h, enc, blob[off:off + dlen]


def read_str_msg(blob):
    off = 4; slen = struct.unpack_from('<I', blob, off)[0]; off += 4
    return blob[off:off + slen - 1].decode('utf-8', 'replace')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('rec_dir')
    ap.add_argument('out_bag')
    ap.add_argument('--stride', type=int, default=1)
    ap.add_argument('--max', type=int, default=0)
    ap.add_argument('--offset-ns', type=int, default=0,
                    help='constant clock offset added to every output timestamp, in ns '
                         '(use to put a second laptop\'s recording on the reference clock)')
    ap.add_argument('--time-source', choices=['assoc', 'color_global'], default='assoc',
                    help="'assoc' (default, unchanged behaviour): use "
                         "associated_host_epoch_timestamp_ns from rgbd_timestamp_associations.json. "
                         "'color_global': use the colour frame's own Global Time from "
                         "/device_0/sensor_1/Color_0/image/metadata. The camera's Global Time is a "
                         "continuously updated device->host mapping and is markedly more accurate "
                         "than the association file's single linear clock_fit; on the 0807 dataset "
                         "it cut the cross-camera scatter from 9.5 ms to 1.1 ms (1 sigma).")
    args = ap.parse_args()

    rec = args.rec_dir.rstrip('/')
    name = os.path.basename(rec)
    db = os.path.join(rec, f'{name}.db3')
    if not os.path.exists(db):  # recorder-named bag (recording_<date>_<time>.db3)
        cands = sorted(f for f in os.listdir(rec) if f.endswith('.db3'))
        if len(cands) != 1:
            print(f"cannot pick a .db3 in {rec}: {cands}"); sys.exit(1)
        db = os.path.join(rec, cands[0])
    assoc = json.load(open(os.path.join(rec, 'rgbd_timestamp_associations.json')))['associations']

    con = sqlite3.connect(db); cur = con.cursor()

    # --time-source color_global: build assoc-index -> colour Global Time (ns)
    global_ns = None
    if args.time_source == 'color_global':
        cur.execute("SELECT id FROM topics WHERE name=?", ('/device_0/sensor_1/Color_0/image/data',))
        cid_topic = cur.fetchone()[0]
        cur.execute("SELECT id FROM topics WHERE name=?", ('/device_0/sensor_1/Color_0/image/metadata',))
        mid_topic = cur.fetchone()[0]
        cur.execute("SELECT id FROM messages WHERE topic_id=? ORDER BY id", (cid_topic,))
        idx_of_cid = {r[0]: i for i, r in enumerate(cur.fetchall())}
        cur.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY id", (mid_topic,))
        gt = []
        for (blob,) in cur.fetchall():
            f = dict(kv.split('=', 1) for kv in read_str_msg(blob).split(';') if '=' in kv)
            gt.append(int(round(float(f['timestamp']) * 1e6)))  # Global Time ms -> ns
        missing = [i for i, a in enumerate(assoc) if a['color_message_id'] not in idx_of_cid]
        if missing:
            print(f"cannot map {len(missing)} colour frames to metadata; aborting"); sys.exit(1)
        global_ns = [gt[idx_of_cid[a['color_message_id']]] for a in assoc]
        drift = [global_ns[i] - int(a['associated_host_epoch_timestamp_ns'])
                 for i, a in enumerate(assoc)]
        d = np.array(drift) / 1e6
        print(f"[{name}] time-source=color_global | vs assoc stamps: "
              f"median {np.median(d):+.2f} ms, sd {d.std():.2f} ms")

    def one(topic):
        cur.execute("SELECT id FROM topics WHERE name=?", (topic,)); r = cur.fetchone()
        if not r: return None
        cur.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (r[0],)); m = cur.fetchone()
        return m[0] if m else None

    dci = parse_camera_info_str(read_str_msg(one('/device_0/sensor_0/Depth_0/camera_info')))
    cci = parse_camera_info_str(read_str_msg(one('/device_0/sensor_1/Color_0/camera_info')))
    fxd, fyd, ppxd, ppyd = float(dci['fx']), float(dci['fy']), float(dci['ppx']), float(dci['ppy'])
    fxc, fyc, ppxc, ppyc = float(cci['fx']), float(cci['fy']), float(cci['ppx']), float(cci['ppy'])
    kc = [float(x) for x in cci['coeffs'].split(',')]  # k1,k2,p1,p2,k3 (Inverse Brown Conrady)
    W, H = int(cci['width']), int(cci['height'])
    R_d2c, t_d2c = parse_tf_ref(read_str_msg(one('/device_0/sensor_1/Color_0/tf/ref_0')))
    du_val = read_str_msg(one('/device_0/sensor_0/option/Depth_Units/value'))
    depth_units = float(du_val)
    print(f"[{name}] color {W}x{H} fxc={fxc:.2f} | depth fxd={fxd:.2f} | baseline t={t_d2c} | depth_units={depth_units}")

    # precompute deprojection grid
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    xnd = (uu - ppxd) / fxd
    ynd = (vv - ppyd) / fyd
    k1, k2, p1, p2, k3 = kc

    def align(depth_mm):  # depth_mm: HxW uint16
        Z = depth_mm.astype(np.float64) * depth_units  # meters
        valid = Z > 0
        X = xnd * Z; Y = ynd * Z
        Xc = R_d2c[0, 0]*X + R_d2c[0, 1]*Y + R_d2c[0, 2]*Z + t_d2c[0]
        Yc = R_d2c[1, 0]*X + R_d2c[1, 1]*Y + R_d2c[1, 2]*Z + t_d2c[1]
        Zc = R_d2c[2, 0]*X + R_d2c[2, 1]*Y + R_d2c[2, 2]*Z + t_d2c[2]
        valid &= Zc > 0
        with np.errstate(divide='ignore', invalid='ignore'):
            x = Xc / Zc; y = Yc / Zc
        r2 = x*x + y*y
        f = 1 + k1*r2 + k2*r2**2 + k3*r2**3
        xd = x*f + 2*p1*x*y + p2*(r2 + 2*x*x)
        yd = y*f + 2*p2*x*y + p1*(r2 + 2*y*y)
        ucf = fxc*xd + ppxc; vcf = fyc*yd + ppyc
        iu = np.round(ucf).astype(np.int32); iv = np.round(vcf).astype(np.int32)
        inb = valid & (iu >= 0) & (iu < W) & (iv >= 0) & (iv < H)
        flat_idx = (iv[inb] * W + iu[inb]).astype(np.int64)
        zmm = np.round(Zc[inb] * 1000.0).astype(np.uint16)
        out = np.full(W*H, 0, dtype=np.uint16)
        # z-buffer: keep nearest (min) depth per target pixel
        order = np.argsort(-zmm.astype(np.int64))  # far first so near overwrites
        out[flat_idx[order]] = zmm[order]
        return out.reshape(H, W)

    # --- ROS2 bag writer ---
    import rclpy.serialization as ser
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from sensor_msgs.msg import Image, CameraInfo
    from builtin_interfaces.msg import Time

    if os.path.exists(args.out_bag):
        print("out bag exists, aborting:", args.out_bag); sys.exit(1)
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.out_bag, storage_id='sqlite3'),
                ConverterOptions('cdr', 'cdr'))
    T_COLOR = '/camera/camera/color/image_raw'
    T_DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
    T_CINFO = '/camera/camera/color/camera_info'
    T_DINFO = '/camera/camera/aligned_depth_to_color/camera_info'
    for tn, tt in [(T_COLOR, 'sensor_msgs/msg/Image'), (T_DEPTH, 'sensor_msgs/msg/Image'),
                   (T_CINFO, 'sensor_msgs/msg/CameraInfo'), (T_DINFO, 'sensor_msgs/msg/CameraInfo')]:
        writer.create_topic(TopicMetadata(name=tn, type=tt, serialization_format='cdr'))

    def make_cinfo(stamp):
        ci = CameraInfo(); ci.header.stamp = stamp; ci.header.frame_id = 'camera_color_optical_frame'
        ci.width = W; ci.height = H; ci.distortion_model = 'plumb_bob'
        ci.d = [0.0]*5
        ci.k = [fxc, 0.0, ppxc, 0.0, fyc, ppyc, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fxc, 0.0, ppxc, 0.0, 0.0, fyc, ppyc, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    n = 0
    for i, a in enumerate(assoc):
        if i % args.stride != 0:
            continue
        if args.max and n >= args.max:
            break
        cur.execute("SELECT data FROM messages WHERE id=?", (a['color_message_id'],)); cm = cur.fetchone()
        cur.execute("SELECT data FROM messages WHERE id=?", (a['depth_message_id'],)); dm = cur.fetchone()
        if not cm or not dm:
            continue
        cw, ch, cenc, cbytes = read_image_msg(cm[0])
        dw, dh, denc, dbytes = read_image_msg(dm[0])
        depth = np.frombuffer(dbytes, dtype=np.uint16).reshape(dh, dw)
        aligned = align(depth)

        base_ns = global_ns[i] if global_ns is not None else int(a['associated_host_epoch_timestamp_ns'])
        ts_ns = base_ns + args.offset_ns
        stamp = Time(sec=ts_ns // 1_000_000_000, nanosec=ts_ns % 1_000_000_000)

        cimg = Image(); cimg.header.stamp = stamp; cimg.header.frame_id = 'camera_color_optical_frame'
        cimg.height = ch; cimg.width = cw; cimg.encoding = 'bgr8'; cimg.is_bigendian = 0
        cimg.step = cw*3; cimg.data = bytes(cbytes)

        dimg = Image(); dimg.header.stamp = stamp; dimg.header.frame_id = 'camera_color_optical_frame'
        dimg.height = H; dimg.width = W; dimg.encoding = '16UC1'; dimg.is_bigendian = 0
        dimg.step = W*2; dimg.data = aligned.tobytes()

        writer.write(T_COLOR, ser.serialize_message(cimg), ts_ns)
        writer.write(T_DEPTH, ser.serialize_message(dimg), ts_ns)
        writer.write(T_CINFO, ser.serialize_message(make_cinfo(stamp)), ts_ns)
        writer.write(T_DINFO, ser.serialize_message(make_cinfo(stamp)), ts_ns)
        n += 1
        if n % 200 == 0:
            print(f"  wrote {n} frames...")
    del writer
    con.close()
    print(f"[{name}] DONE: {n} synchronized aligned RGB-D frames -> {args.out_bag}")


if __name__ == '__main__':
    main()
