# ORB-SLAM3 CPU / CUDA descriptor experiment

This optional backend runs **ORB descriptor sampling on CUDA**. Keypoint detection,
octree distribution, orientation, Gaussian blur, matching, tracking, bundle
adjustment and IMU preintegration remain CPU. It is hybrid SLAM, not a full GPU
port. Generic commands retain the CPU default; the two split entrypoints now
default to `--orb-backend cuda`, with `--orb-backend cpu` available for comparison.
Selecting CUDA never silently falls back to CPU.

The remote RTX 5090 Laptop supports `sm_120`. Its installed OpenCV has no CUDA
modules, so the small CUDA plugin uses the already-installed CUDA 12.8 compiler
without replacing OpenCV or the ROS environment. It preserves the original
descriptor pattern and CPU-selected keypoints. GPU upload/download and kernel
launch overhead can outweigh the accelerated sampling; report measured ratios
even when CUDA is slower.

If input is processed correctly but ORB exports poses for fewer than 90% of frames, the report says
`NOT_TRACKING` and the command exits 2 after collecting both comparison runs.
Such timing measures uninitialized processing, not successful SLAM throughput.
The experiment does not lower ORB's feature or motion initialization gates.

## Build / exact-descriptor check (remote laptop)

Stop active ORB processes before rebuilding its library. In `slam-codex`:

```bash
cd /home/jucpark/sam6d_object_memory
source /home/jucpark/anaconda3/etc/profile.d/conda.sh
conda activate realsense
python integration/run_orb_slam_gpu.py --build-cuda
cmake --build orbslam_ws/build_jazzy/orbslam3_core --target ORB_SLAM3 -j 2
cmake --build orbslam_ws/build_jazzy/orbslam3_core --target check_cuda_descriptors -j 2
export ORB_SLAM3_CUDA_LIBRARY="$PWD/orbslam_ws/src/ORB_SLAM3/lib/liborb_cuda_descriptors.so"
orbslam_ws/build_jazzy/orbslam3_core/check_cuda_descriptors
```

The check asserts identical keypoints, orientations, octaves and every descriptor
byte, then reports full feature-extraction times. An optional grayscale image
path tests a captured scene. This microbenchmark does not measure whole-SLAM FPS.

## Same-input full-SLAM comparison

Use a camera-calibrated settings file matching the bag's RGB stream. The runner
uses an isolated ROS domain, replays exactly the requested existing bag interval,
drains input, shuts ORB down, and retains logs and trajectories. It does not record
new images or acquire cameras. Each run uses a fresh output directory.

```bash
python integration/run_orb_slam_gpu.py \
  --compare --settings /absolute/path/to/orb_settings.yaml \
  --bag output/split_static_9pin_20260909_000026/distributed/slam/capture_raw \
  --start-offset 15 --seconds 60 --output output/orb_cpu_cuda_comparison
```

`comparison.json` reports input timestamp equality, consumed frames, trajectory
coverage, processing/queue latency and median speedup (`CPU / CUDA`; below 1 means
CUDA is slower). Timing excludes the first 30 consumed frames; shutdown's separate
`TrackRGBD` timing includes warmup. A stationary sequence verifies execution and
stationary tracking only; rotational accuracy needs a moving sequence and ground
truth. Compare repeated/reversed orders before interpreting small differences.

For older bags containing separate images, also pass
`--rgb-topic /camera/slam_camera/color/image_raw --depth-topic /camera/slam_camera/aligned_depth_to_color/image_raw`.
This uses the existing approximate image synchronizer; the timestamp-equality
check still rejects interpreting different paired inputs as a fair speed ratio.

The original 2026-09-09 static replay is recorded in
`output/gpu_native_comparison_20260909/summary.md`: both backends tracked all 1,799
frames; processing medians were CPU 8.652 ms and CUDA 9.156 ms (CUDA 5.81% slower).
This earlier measurement did not demonstrate a GPU speed benefit.

## Split defaults, remote viewer and batched descriptors (2026-09-09)

`create_map_urdf_split.py` opens the remote SLAM desktop's native ORB map and
current-frame windows during capture and calibration replay by default. Use
`--headless` to disable both, or `--view` to explicitly request them. Input-only
checks do not start ORB. `--from-capture`/`--resume` propagate the replay viewer
and backend choices. The existing user desktop is discovered through its user
session; X11 access control is not disabled.

Both split entrypoints, including recorded realtime replay, explicitly select
CUDA descriptors. Runtime preflight checks the core hook, plugin and device;
the ORB process log must also attest actual CUDA execution. The generic ROS
launch accepts `runtime.descriptor_backend`, and only the ORB process receives
the backend environment. This does not enable a remote ORB viewer for realtime
ObjectMemory; its normal overlay remains on the SAM laptop.

The CPU still computes each pyramid level's original blur and keypoints. These
images are packed into one atlas for one CUDA descriptor call per frame, then
descriptors are scattered in the original mono/stereo order. This reduces eight
octave calls to one without changing the CUDA plugin ABI or descriptor math.
The existing `--fmad=false` compiler option remains in effect. Batching addresses
the per-transfer overhead described in the
[NVIDIA CUDA best-practices guide](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html).

Remote GPU verification passed 12 image/size/overlap cases with identical
descriptor bytes and keypoints. On the captured 580-feature image, feature
extraction median decreased from 4.327 ms (prior CUDA) to 3.036 ms (batched CUDA);
CPU measured 3.033 ms in the latter check. Whole-SLAM replay results varied with
order: CPU/CUDA medians were 8.715/9.259 ms, and CUDA/CPU reverse-order runs were
8.800/9.134 ms. All six before/after/reverse runs processed and tracked the same
899 frames without input errors. **A repeatable whole-SLAM speedup or improved
pose accuracy has not been demonstrated.** Descriptor equality preserves the
feature contract; there is no ground-truth trajectory in this stationary input.

The actual remote viewer smoke test opened both windows on `DISPLAY=:1`, tracked
600 recorded frames with CUDA, and exited cleanly. Full logs, settings, source
backups, benchmark JSON and the verification report are under
[`output/remote_viewer_cuda_1788920426738`](../output/remote_viewer_cuda_1788920426738/).

## Consume an already-running camera

```bash
python integration/run_orb_slam_gpu.py --backend cuda \
  --settings /absolute/path/to/orb_settings.yaml \
  --domain-id 73 --seconds 60 --output output/orb_live_cuda
```

The camera must already publish `/camera/slam_camera/rgbd` in the selected domain.
This command does not reconfigure sync modes. Choose `--backend cpu` for the
original implementation. Use `--localization` only with settings containing a
valid `System.LoadAtlasFromFile` for the same camera calibration.

Relative Atlas names resolve against the original settings file's directory.
Place its existing `<name>.osa` file or symlink there. The runner validates the
file and stages an `input_atlas.osa` symlink in its own working directory, because
the core opens `./<name>.osa`. Absolute Atlas paths are accepted and staged the
same way. A missing Atlas fails before ORB starts. Actual relative-path CUDA
localization passed 827/828 poses in `output/gpu_relative_atlas_20260909`.

For a self-contained single-camera run (the actual live smoke-test entrypoint):

```bash
python integration/run_orb_slam_gpu.py --backend cuda --start-camera \
  --serial 253822302376 --sync-mode 1 --domain-id 73 --seconds 60 \
  --settings output/gpu_native_comparison_20260909/settings.yaml \
  --output output/orb_live_cuda_manual_01
```

This validates the exact serial and profile, starts/stops its own camera, and
does not record a bag. Fresh mapping uses the actual CameraInfo calibration in
`live_settings.yaml`, with changes recorded in `calibration_delta.json`.
Loading a saved Atlas keeps strict calibration matching. An existing ORB node
in the chosen ROS domain blocks the experiment to avoid draining another run.

Actual live tests on 2026-09-09 consumed all inputs on CPU and CUDA but repeatedly
lost tracking in an almost black RGB scene. The corrected CUDA 10-second retest
reported 326 inputs, 1 pose, `NOT_TRACKING`, exit 2, and a clean camera stop.
This is a verified failure under that input condition, not successful live SLAM.
The lit recorded sequence's CPU/CUDA runs both succeeded; see the detailed report
and unmodified lit/dark sample images before drawing an acceleration conclusion.

The independent `run_orb_slam_imu.py` launcher supplies IMU calibration/settings
and `--use-imu`; the same descriptor backend works with IMU_RGBD. IMU processing
still runs on CPU, and successful visual tracking does not prove inertial
initialization. A stationary camera may never excite the initializer sufficiently.
