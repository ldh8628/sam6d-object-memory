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
        # 실제 계수를 싣는다. 'plumb_bob' 은 같은 5계수 모델의 ROS 이름이고,
        # align() 이 이미 이 계수로 depth 를 왜곡된 color 격자에 얹었으므로
        # 소비자가 이 값으로 undistort 하면 depth 와 어긋난다는 점에 주의.
        ci.d = [float(x) for x in kc]
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

        # --- 입력 검증 (첫 프레임에서 한 번) ---------------------------------
        # 예전에는 cenc/denc 를 파싱해놓고 버린 뒤 무조건 'bgr8' 로 단정했다.
        # rgb8 이 들어오면 R/B 가 뒤바뀐 채 아무도 눈치채지 못한 채 통과한다.
        if n == 0:
            if cenc not in ('bgr8', 'rgb8'):
                print(f"color 인코딩이 bgr8/rgb8 이 아니다: {cenc!r} — 중단"); sys.exit(1)
            if denc not in ('mono16', '16UC1'):
                print(f"depth 인코딩이 16bit 가 아니다: {denc!r} — 중단"); sys.exit(1)
            if (cw, ch) != (W, H):
                print(f"color 이미지 {cw}x{ch} != camera_info {W}x{H} — 중단"); sys.exit(1)
            if (dw, dh) != (W, H):
                print(f"depth {dw}x{dh} != color {W}x{H} — 중단 "
                      f"(정렬 격자가 color 해상도로 만들어져 있다)"); sys.exit(1)
            print(f"[{name}] 입력 검증 OK: color={cenc} depth={denc} {W}x{H}")
        if len(cbytes) != cw*ch*3:
            print(f"color 바이트 수 이상: {len(cbytes)} != {cw*ch*3} (frame {i}) — 중단")
            sys.exit(1)
        if len(dbytes) != dw*dh*2:
            print(f"depth 바이트 수 이상: {len(dbytes)} != {dw*dh*2} (frame {i}) — 중단")
            sys.exit(1)

        depth = np.frombuffer(dbytes, dtype=np.uint16).reshape(dh, dw)
        aligned = align(depth)

        base_ns = global_ns[i] if global_ns is not None else int(a['associated_host_epoch_timestamp_ns'])
        ts_ns = base_ns + args.offset_ns
        stamp = Time(sec=ts_ns // 1_000_000_000, nanosec=ts_ns % 1_000_000_000)

        cimg = Image(); cimg.header.stamp = stamp; cimg.header.frame_id = 'camera_color_optical_frame'
        cimg.height = ch; cimg.width = cw; cimg.encoding = 'bgr8'; cimg.is_bigendian = 0
        cimg.step = cw*3
        if cenc == 'rgb8':                       # 선언과 실제를 맞춘다
            cimg.data = np.frombuffer(cbytes, np.uint8).reshape(ch, cw, 3)[:, :, ::-1].tobytes()
        else:
            cimg.data = bytes(cbytes)

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

    # 변환 조건을 bag 옆에 남긴다. 예전에는 stride/offset/time-source 가 어디에도
    # 기록되지 않아 잘린 bag 과 전체 bag 을 구분할 방법이 없었다.
    info = {
        'source': os.path.abspath(rec),
        'db3': os.path.basename(db),
        'frames_written': n,
        'associations': len(assoc),
        'stride': args.stride,
        'max': args.max,
        'time_source': args.time_source,
        'offset_ns': args.offset_ns,
        'color_size': [W, H],
        'color_encoding': cenc if n else None,
        'depth_encoding': denc if n else None,
        'depth_units': depth_units,
        'baseline_depth_to_color_m': float(t_d2c[0]),
        'K': [fxc, 0.0, ppxc, 0.0, fyc, ppyc, 0.0, 0.0, 1.0],
        'D': [float(x) for x in kc],
        'distortion_model_source': 'Inverse Brown Conrady',
        'topics': [T_COLOR, T_DEPTH, T_CINFO, T_DINFO],
    }
    with open(os.path.join(args.out_bag, 'conversion_info.json'), 'w') as f:
        json.dump(info, f, indent=1)

    expected = len(range(0, len(assoc), args.stride))
    if args.max:
        expected = min(expected, args.max)
    if n == 0:
        print(f"[{name}] 프레임을 하나도 쓰지 못했다 — 실패"); sys.exit(1)
    if n < expected:
        print(f"[{name}] 경고: {expected - n} 프레임이 누락됐다 ({n}/{expected})")
    print(f"[{name}] DONE: {n} synchronized aligned RGB-D frames -> {args.out_bag}")
    print(f"[{name}] baseline(depth->color) = {t_d2c[0]:+.6f} m | D = "
          + ', '.join(f'{x:+.6f}' for x in kc))


if __name__ == '__main__':
    main()
