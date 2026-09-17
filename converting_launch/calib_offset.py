#!/usr/bin/env python3
"""두 카메라(SLAM 정면 / SAM 측면)의 호스트 시계 오프셋을 실측한다.

두 D455F 가 하드웨어 genlock(master/slave) 되어 있으면 **같은 인덱스의 프레임은
물리적으로 같은 순간에 노출**된다. 따라서 두 bag 의 Global Time 차이는 전부
'호스트 시계차 + 카메라별 매핑 지연' 이며, 그 중앙값을 빼면 공통 시간축이 된다.

peer_timestamp_comparison.json 의 probe 값(NTP RTT 기반, 불확실도 ~1.4 ms)이 있으면
초기 추정으로 쓰고, 실제 프레임으로 잔차를 재서 보정한다. 없으면 probe=0 에서
프레임만으로 추정하되 |offset| < 0.5 * (탐색창) 인 경우에만 신뢰한다.

stdout 에는 오프셋(ns) 정수 하나만 찍는다 — 스크립트가 받아쓰기 위함.
진단은 전부 stderr.

사용:
    python3 calib_offset.py --slam:=<SLAM 원본 폴더> --sam:=<SAM 원본 폴더>
                           [--stream:=color|depth] [--cache:=<경로>] [--force]
"""
import sqlite3, json, os, sys, argparse, struct
import numpy as np

TOPIC = {'color': '/device_0/sensor_1/Color_0/image/metadata',
         'depth': '/device_0/sensor_0/Depth_0/image/metadata'}


def ros2_argv(argv):
    """ROS2 스타일 `--key:=value` 를 argparse 가 아는 `--key value` 로 편다."""
    out = []
    for a in argv:
        if a.startswith('--') and ':=' in a:
            k, v = a.split(':=', 1)
            out += [k, v]
        else:
            out.append(a)
    return out


def read_str(b):
    ln = struct.unpack_from('<I', b, 4)[0]
    return b[8:8 + ln].rstrip(b'\x00').decode('utf-8', 'replace')


def pick_db3(sess_dir):
    c = sorted(f for f in os.listdir(sess_dir) if f.endswith('.db3'))
    if len(c) != 1:
        raise RuntimeError(f'{sess_dir}: .db3 를 하나로 특정할 수 없다 -> {c}')
    return os.path.join(sess_dir, c[0])


def global_times(sess_dir, stream):
    """SDK bag 의 프레임별 Global Time(ms). metadata 토픽이 없으면 None."""
    con = sqlite3.connect(f'file:{pick_db3(sess_dir)}?mode=ro', uri=True)
    try:
        row = con.execute("select id from topics where name=?", (TOPIC[stream],)).fetchone()
        if row is None:
            return None
        out = []
        for (blob,) in con.execute(
                "select data from messages where topic_id=? order by id", (row[0],)):
            f = dict(kv.split('=', 1) for kv in read_str(blob).split(';') if '=' in kv)
            if 'timestamp' in f:
                out.append(float(f['timestamp']))
        return np.array(out) if out else None
    finally:
        con.close()


def nearest(a, b):
    j = np.clip(np.searchsorted(b, a), 1, len(b) - 1)
    c = np.stack([b[j - 1], b[j]])
    k = np.argmin(np.abs(c - a), axis=0)
    return c[k, np.arange(len(a))] - a


def peer_probe_ms(slam_dir):
    """레코더가 촬영 당시 기록한 NTP 기반 호스트 시계차(remote - local, ms).

    사이드카 이름이 데이터셋마다 다르다.
      0807: peer_timestamp_comparison.json -> clock_mapping.remote_minus_local_clock_ns
      260804: session_manifest.json        -> peer_clock.remote_minus_local_clock_ns
    """
    p = os.path.join(slam_dir, 'peer_timestamp_comparison.json')
    if os.path.isfile(p):
        j = json.load(open(p, encoding='utf-8'))
        return (j['clock_mapping']['remote_minus_local_clock_ns'] / 1e6,
                j.get('verdict'), 'peer_timestamp_comparison')
    p = os.path.join(slam_dir, 'session_manifest.json')
    if os.path.isfile(p):
        try:
            j = json.load(open(p, encoding='utf-8'))
        except ValueError:
            j = {}
        pc = j.get('peer_clock') or {}
        if isinstance(pc.get('remote_minus_local_clock_ns'), (int, float)):
            return (pc['remote_minus_local_clock_ns'] / 1e6,
                    f"probe_count={pc.get('clock_probe_count')}", 'session_manifest.peer_clock')
    return None, None, None


FRAME_MS = 1000.0 / 30.0          # 30 Hz


def calibrate(slam_dir, sam_dir, stream='color'):
    """probe(NTP 실측) 를 기준으로 두고, genlock 프레임으로 잔차만 다듬는다.

    ★ probe 가 없으면 실패로 돌린다. 최근접 프레임 매칭은 실제 오프셋이 6 초든 6 밀리초든
    언제나 ±16.7 ms 안의 값을 돌려주기 때문에, probe 없이 이 방법을 쓰면 '작고 그럴듯한
    거짓값' 이 나온다 (260804 에서 실제 -6.607 s 를 -6.36 ms 로 오판했다).
    """
    probe, verdict, probe_src = peer_probe_ms(slam_dir)
    if probe is None:
        return {'ok': False,
                'reason': 'NTP probe 사이드카가 없다 — 최근접 프레임 매칭만으로는 '
                          '오프셋을 알 수 없다 (항상 ±16.7 ms 안의 거짓값이 나온다)',
                'probe_ms': None, 'peer_verdict': None}

    M = global_times(slam_dir, stream)
    S = global_times(sam_dir, stream)
    if M is None or S is None:
        # probe 만으로도 쓸 수 있다. genlock 잔차 보정만 못 할 뿐이다.
        return {'ok': True, 'stream': stream,
                'probe_ms': probe, 'residual_ms': 0.0,
                'total_offset_ms': probe, 'total_offset_ns': int(round(probe * 1e6)),
                'sd_ms': None, 'n': 0, 'peer_verdict': verdict,
                'probe_source': probe_src,
                'method': 'peer_clock_probe',
                'note': f'Global Time metadata 토픽 없음 '
                        f'(SLAM={"O" if M is not None else "X"}, '
                        f'SAM={"O" if S is not None else "X"}) — probe 값만 쓴다'}

    d = nearest(M, S - probe)
    med = float(np.median(d))
    keep = np.abs(d - med) < 16.0            # 프레임 되감기·중복 제외
    if keep.sum() < 30:
        return {'ok': False, 'reason': f'유효 프레임 쌍 부족 (n={int(keep.sum())})',
                'probe_ms': probe, 'peer_verdict': verdict}
    resid = float(np.median(d[keep]))
    total = probe + resid
    r = {'ok': True, 'stream': stream,
         'probe_ms': probe, 'residual_ms': resid,
         'total_offset_ms': total, 'total_offset_ns': int(round(total * 1e6)),
         'sd_ms': float(d[keep].std()), 'n': int(keep.sum()),
         'peer_verdict': verdict, 'probe_source': probe_src,
         'method': 'calib_offset_genlock'}
    if abs(resid) > FRAME_MS / 2.0:
        # 잔차가 반 프레임을 넘으면 그건 '다듬기' 가 아니라 프레임 대응이 틀렸다는 뜻이다.
        r['ok'] = False
        r['reason'] = (f'genlock 잔차 {resid:+.2f} ms 가 반 프레임({FRAME_MS / 2:.1f} ms) 을 '
                       f'넘는다 — probe 가 틀렸거나 두 카메라가 genlock 이 아니다')
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slam', required=True, help='SLAM(정면) 카메라 SDK 원본 폴더')
    ap.add_argument('--sam', required=True, help='SAM(측면) 카메라 SDK 원본 폴더')
    ap.add_argument('--stream', choices=['color', 'depth'], default='color')
    ap.add_argument('--cache', default='')
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args(ros2_argv(sys.argv[1:]))

    key = f'{os.path.abspath(a.slam)}|{os.path.abspath(a.sam)}|{a.stream}'
    cache = {}
    if a.cache and os.path.exists(a.cache):
        cache = json.load(open(a.cache, encoding='utf-8'))
    if key in cache and not a.force:
        r = cache[key]
    else:
        r = calibrate(a.slam, a.sam, a.stream)
        if a.cache:
            cache[key] = r
            json.dump(cache, open(a.cache, 'w'), indent=1, ensure_ascii=False)

    if not r.get('ok'):
        print(f"calib_offset 실패: {r.get('reason')}", file=sys.stderr)
        print(json.dumps(r, ensure_ascii=False), file=sys.stderr)
        return 1
    print(f"stream={r['stream']}  peer_verdict={r['peer_verdict']}", file=sys.stderr)
    probe_s = 'n/a' if r['probe_ms'] is None else f"{r['probe_ms']:.2f} ms"
    print(f"  probe {probe_s}  +  실측잔차 {r['residual_ms']:+.2f} ms"
          f"  =>  {r['total_offset_ms']:.2f} ms   (1σ {r['sd_ms']:.2f} ms, n={r['n']})",
          file=sys.stderr)
    print(r['total_offset_ns'])          # stdout 은 오프셋(ns)만
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
