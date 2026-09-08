#!/usr/bin/env python3
"""Preview two RealSense cameras, choose their roles, then publish ROS 2 topics."""
from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import shlex
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

CAMERA_NAMESPACE = "camera"
SLAM_CAMERA = "slam_camera"
SAM_CAMERA = "sam_camera"
CONDA_SH = Path.home() / "miniconda3/etc/profile.d/conda.sh"


def runtime_domain(value: int | None = None) -> int:
    if value is None:
        return 100 + secrets.randbelow(101)
    if not 0 <= value <= 232:
        raise ValueError("ROS domain ID must be between 0 and 232")
    return value


def _conda_command(environment: str, command: list[str], domain_id: int = 72) -> list[str]:
    script = "; ".join([
        "unset PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION",
        "unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH",
        f"source {shlex.quote(str(CONDA_SH))}",
        f"conda activate {shlex.quote(environment)}",
        f"export ROS_DOMAIN_ID={int(domain_id)}",
        "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST",
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
        f"export FASTRTPS_DEFAULT_PROFILES_FILE={shlex.quote(str(Path(__file__).resolve().parent / 'fastdds_input.xml'))}",
        shlex.join(command),
    ])
    return ["bash", "-lc", script]


def _request_shutdown(_signum, _frame) -> None:
    raise KeyboardInterrupt


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def _stop(process: subprocess.Popen | None,
          timeouts: tuple[float, float, float] = (30.0, 5.0, 2.0)) -> bool:
    if process is None:
        return True
    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        process.poll()
        if not _group_alive(process.pid):
            return True
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while _group_alive(process.pid) and time.monotonic() < deadline:
            process.poll()
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                continue
    process.poll()
    return not _group_alive(process.pid)


@dataclass(frozen=True)
class CameraDevice:
    serial: str
    video: str
    label: str


def topics(camera: str) -> dict[str, str]:
    base = f"/{CAMERA_NAMESPACE}/{camera}"
    return {
        "rgbd": f"{base}/rgbd",
        "rgb_metadata": f"{base}/color/metadata",
        "depth_metadata": f"{base}/depth/metadata",
        "rgb": f"{base}/color/image_raw",
        "depth": f"{base}/aligned_depth_to_color/image_raw",
        "caminfo": f"{base}/color/camera_info",
        "depth_caminfo": f"{base}/aligned_depth_to_color/camera_info",
    }


def serial(value: str) -> str:
    value = value.strip().strip("'\"").removeprefix("_")
    if not value:
        raise ValueError("camera serial must not be empty")
    return value


def parse_v4l2_devices(text: str) -> list[tuple[str, list[str]]]:
    devices: list[tuple[str, list[str]]] = []
    label, nodes = "", []
    for line in text.splitlines():
        if line and not line[0].isspace():
            if label:
                devices.append((label.rstrip(":"), nodes))
            label, nodes = line.strip(), []
        elif line.strip().startswith("/dev/video"):
            nodes.append(line.strip())
    if label:
        devices.append((label.rstrip(":"), nodes))
    return devices


def parse_udev_properties(text: str) -> dict[str, str]:
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def parse_realsense_serials(text: str) -> dict[str, str]:
    aliases: dict[str, str] = {}
    for section in text.split("Device info:")[1:]:
        values = {}
        for line in section.splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
        camera_serial = values.get("Serial Number", "").strip()
        asic_serial = values.get("Asic Serial Number", "").strip()
        if camera_serial:
            aliases[camera_serial] = camera_serial
            if asic_serial:
                aliases[asic_serial] = camera_serial
    return aliases


def discover_cameras() -> list[CameraDevice]:
    enumerated = subprocess.run(
        _conda_command("realsense", ["rs-enumerate-devices"]), check=True,
        capture_output=True, text=True, timeout=10)
    serial_aliases = parse_realsense_serials(enumerated.stdout)
    listing = subprocess.run(
        ["v4l2-ctl", "--list-devices"], check=True, capture_output=True, text=True,
        timeout=5)
    found: dict[str, CameraDevice] = {}
    for label, nodes in parse_v4l2_devices(listing.stdout):
        for node in nodes:
            udev = subprocess.run(
                ["udevadm", "info", "--query=property", f"--name={node}"],
                check=True, capture_output=True, text=True, timeout=5)
            props = parse_udev_properties(udev.stdout)
            udev_serial = props.get("ID_SERIAL_SHORT", "").strip()
            camera_serial = serial_aliases.get(udev_serial, "")
            product = props.get("ID_V4L_PRODUCT", label).replace("_", " ")
            if not camera_serial or "realsense" not in product.lower():
                continue
            formats = subprocess.run(
                ["v4l2-ctl", "--device", node, "--list-formats-ext"],
                capture_output=True, text=True, timeout=5)
            if formats.returncode == 0 and any(
                    fmt in formats.stdout for fmt in ("YUYV", "MJPG", "RGB3")):
                found.setdefault(camera_serial, CameraDevice(camera_serial, node, label))
                break
    return sorted(found.values(), key=lambda device: device.video)


def role_contract(slam_serial: str, sam_serial: str) -> dict[str, str]:
    slam_serial, sam_serial = serial(slam_serial), serial(sam_serial)
    if slam_serial == sam_serial:
        raise ValueError("SLAM and SAM must use different camera serials")
    return {"slam_serial": slam_serial, "sam_serial": sam_serial}


def choose_roles(devices: list[CameraDevice]) -> dict[str, str] | None:
    if len(devices) != 2:
        raise ValueError(f"exactly two RealSense RGB devices are required (found {len(devices)})")

    # Ubuntu's apt OpenCV must win over an incompatible /usr/local NumPy install.
    dist_packages = "/usr/lib/python3/dist-packages"
    if Path(dist_packages).is_dir() and dist_packages not in sys.path[:1]:
        sys.path.insert(0, dist_packages)
    import cv2
    import tkinter as tk

    captures = [cv2.VideoCapture(device.video, cv2.CAP_V4L2) for device in devices]
    for capture in captures:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not all(capture.isOpened() for capture in captures):
        for capture in captures:
            capture.release()
        raise RuntimeError("failed to open both V4L2 RGB preview devices")

    def show_window() -> dict[str, str] | None:
        root = tk.Tk()
        root.title("D455f camera roles")
        selected = tk.StringVar(value=devices[0].serial)
        result: dict[str, str] | None = None
        panels = []
        for column, device in enumerate(devices):
            panel = tk.Label(root, text=device.video)
            panel.grid(row=0, column=column, padx=6, pady=6)
            panels.append(panel)
            tk.Radiobutton(
                root, text=f"SLAM  {device.serial}  ({device.video})",
                variable=selected, value=device.serial).grid(row=1, column=column)

        def update() -> None:
            if not root.winfo_exists():
                return
            for capture, panel in zip(captures, panels):
                ok, frame = capture.read()
                if not ok:
                    continue
                frame = cv2.resize(frame, (640, 480))
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ppm = (f"P6\n{rgb.shape[1]} {rgb.shape[0]}\n255\n".encode()
                       + rgb.tobytes())
                image = tk.PhotoImage(data=ppm, format="PPM")
                panel.configure(image=image, width=640, height=480)
                panel.image = image
            root.after(33, update)

        def confirm() -> None:
            nonlocal result
            slam = selected.get()
            sam = next(device.serial for device in devices if device.serial != slam)
            result = role_contract(slam, sam)
            root.destroy()

        buttons = tk.Frame(root)
        buttons.grid(row=2, column=0, columnspan=2, pady=8)
        tk.Button(buttons, text="확인", width=12,
                  command=confirm).pack(side=tk.LEFT, padx=4)
        tk.Button(buttons, text="취소", width=12,
                  command=root.destroy).pack(side=tk.LEFT, padx=4)
        root.protocol("WM_DELETE_WINDOW", root.destroy)
        root.after(0, update)
        root.mainloop()
        return result

    try:
        return show_window()
    except Exception as exc:
        raise RuntimeError(f"camera preview failed: {exc}") from exc
    finally:
        for capture in captures:
            capture.release()


def camera_command(camera_serial: str, camera: str, domain_id: int,
                   hardware_sync_mode: int = 0) -> list[str]:
    return _conda_command("realsense", [
        "ros2", "launch", "realsense2_camera", "rs_launch.py",
        f"serial_no:=_{serial(camera_serial)}",
        f"camera_namespace:={CAMERA_NAMESPACE}",
        f"camera_name:={camera}",
        "align_depth.enable:=true",
        "enable_sync:=true",
        "enable_rgbd:=true",
        "rgb_camera.color_profile:=640x480x30",
        "depth_module.depth_profile:=848x480x30",
        f"depth_module.inter_cam_sync_mode:={hardware_sync_mode}",
        "pointcloud.enable:=false",
        "diagnostics_period:=1.0",
        # The launch default (5s) can terminate RealSense while its sensors close.
        "sigterm_timeout:=25",
    ], domain_id=domain_id)


def wait_camera_frame(topic: str, domain_id: int, timeout: float,
                      processes: list[tuple[str, subprocess.Popen]]) -> None:
    command = _conda_command("realsense", [
        "ros2", "topic", "echo", "--no-daemon", "--once",
        "--qos-reliability", "best_effort", topic, ("realsense2_camera_msgs/msg/RGBD" if topic.endswith("/rgbd")
                else "sensor_msgs/msg/Image"),
    ], domain_id=domain_id)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
        try:
            if probe.wait(timeout=min(5.0, max(0.1, deadline - time.monotonic()))) == 0:
                return
        except subprocess.TimeoutExpired:
            pass
        finally:
            _stop(probe, timeouts=(0.5, 0.5, 0.5))
        failed = [(name, process.returncode) for name, process in processes
                  if process.poll() is not None]
        if failed:
            raise RuntimeError(f"process exited before camera input: {failed}")
    raise TimeoutError(f"no RealSense image received: {topic}")


def write_roles(path: Path, roles: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(roles, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def run(roles: dict[str, str], args: argparse.Namespace) -> None:
    if args.roles_out:
        write_roles(args.roles_out.resolve(), roles)
    sync_modes = (1, 2) if args.hardware_sync else (0, 0)
    processes: list[tuple[str, subprocess.Popen]] = []
    try:
        for name, camera_serial, camera, sync_mode in (
            ("RealSense SLAM", roles["slam_serial"], SLAM_CAMERA, sync_modes[0]),
            ("RealSense SAM", roles["sam_serial"], SAM_CAMERA, sync_modes[1]),
        ):
            processes.append((name, subprocess.Popen(camera_command(
                camera_serial, camera, args.ros_domain_id, sync_mode),
                start_new_session=True)))
            for key in ("rgb", "depth"):
                wait_camera_frame(topics(camera)[key], args.ros_domain_id,
                                  args.camera_timeout, processes)
        print(json.dumps(roles), flush=True)
        print("RealSense publishers READY; Ctrl-C로 종료합니다.", flush=True)
        while True:
            failed = [(name, process.returncode) for name, process in processes
                      if process.poll() is not None]
            if failed:
                raise RuntimeError(f"camera publisher exited: {failed}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        with ThreadPoolExecutor(max_workers=2) as executor:
            stopped = list(executor.map(_stop, (process for _name, process in processes)))
        if not all(stopped):
            raise RuntimeError("RealSense child process cleanup failed")


def self_test() -> None:
    listing = """Intel(R) RealSense(TM) D455f (usb-1):
\t/dev/video2
\t/dev/video3

Intel(R) RealSense(TM) D455f (usb-2):
\t/dev/video8
"""
    assert parse_v4l2_devices(listing) == [
        ("Intel(R) RealSense(TM) D455f (usb-1)", ["/dev/video2", "/dev/video3"]),
        ("Intel(R) RealSense(TM) D455f (usb-2)", ["/dev/video8"]),
    ]
    assert parse_udev_properties("ID_SERIAL_SHORT=123\nID_V4L_PRODUCT=Intel_RealSense\n") \
        == {"ID_SERIAL_SHORT": "123", "ID_V4L_PRODUCT": "Intel_RealSense"}
    assert parse_realsense_serials("""Device info:
    Serial Number                 : 253822302376
    Physical Port                 : 6-3-2
    Asic Serial Number            : 254343064132
""") == {"253822302376": "253822302376", "254343064132": "253822302376"}
    assert role_contract("'_123'", "456") == {
        "slam_serial": "123", "sam_serial": "456"}
    try:
        role_contract("123", "_123")
    except ValueError:
        pass
    else:
        raise AssertionError("duplicate serial accepted")
    assert topics(SLAM_CAMERA)["rgb"] == "/camera/slam_camera/color/image_raw"
    assert topics(SAM_CAMERA)["rgbd"] == "/camera/sam_camera/rgbd"
    command = camera_command("123", SLAM_CAMERA, 72)
    assert "serial_no:=_123" in command[2]
    assert "camera_name:=slam_camera" in command[2]
    assert "ROS_DOMAIN_ID=72" in command[2]
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" in command[2]
    assert 100 <= runtime_domain() <= 200
    assert runtime_domain(7) == 7
    try:
        runtime_domain(233)
    except ValueError:
        pass
    else:
        raise AssertionError("invalid ROS domain accepted")


def main() -> int:
    signal.signal(signal.SIGTERM, _request_shutdown)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, _request_shutdown)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ros-domain-id", type=int, default=72)
    parser.add_argument("--hardware-sync", action="store_true")
    parser.add_argument("--slam-serial")
    parser.add_argument("--sam-serial")
    parser.add_argument("--roles-out", type=Path)
    parser.add_argument("--camera-timeout", type=float, default=30.0)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    self_test()
    if args.self_test:
        print("self-test: PASS")
        return 0
    if (args.slam_serial is None) != (args.sam_serial is None):
        parser.error("--slam-serial and --sam-serial must be provided together")
    if not math.isfinite(args.camera_timeout) or args.camera_timeout <= 0:
        parser.error("--camera-timeout must be positive")
    try:
        devices = discover_cameras()
        if args.slam_serial is not None:
            roles = role_contract(args.slam_serial, args.sam_serial)
            connected = {device.serial for device in devices}
            missing = set(roles.values()) - connected
            if missing:
                raise ValueError(f"camera serial not found in V4L2: {sorted(missing)}")
        else:
            roles = choose_roles(devices)
            if roles is None:
                print("camera selection cancelled", file=sys.stderr)
                return 1
        run(roles, args)
    except (FileNotFoundError, OSError, ValueError, RuntimeError, TimeoutError,
            subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
