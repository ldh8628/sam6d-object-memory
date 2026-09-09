import blenderproc as bproc

import argparse
import os

import bpy
import cv2
import numpy as np
import trimesh


parser = argparse.ArgumentParser()
parser.add_argument("--cad_path", required=True)
parser.add_argument("--output_dir", required=True)
parser.add_argument("--normalize", default=True)
parser.add_argument("--samples", type=int, default=16)
args = parser.parse_args()

render_dir = os.path.abspath("sam6d_master/SAM-6D/Render")
cnos_cam_fpath = os.path.join(
    render_dir,
    "../Instance_Segmentation_Model/utils/poses/predefined_poses/cam_poses_level0.npy",
)

bproc.init()


def get_norm_info(mesh_path):
    mesh = trimesh.load(mesh_path, force="mesh")
    model_points = trimesh.sample.sample_surface(mesh, 1024)[0].astype(np.float32)
    min_value = np.min(model_points, axis=0)
    max_value = np.max(model_points, axis=0)
    radius = max(np.linalg.norm(max_value), np.linalg.norm(min_value))
    return 1 / (2 * radius)


scale = get_norm_info(args.cad_path) if args.normalize else 1
obj = bproc.loader.load_obj(args.cad_path)[0]
obj.set_scale([scale, scale, scale])
obj.set_cp("category_id", 1)

cam_pose = np.load(cnos_cam_fpath)[0].copy()
cam_pose[:3, 1:3] = -cam_pose[:3, 1:3]
cam_pose[:3, -1] = cam_pose[:3, -1] * 0.001 * 2

bpy.context.scene.frame_start = 1
bpy.context.scene.frame_end = 1
bproc.camera.add_camera_pose(cam_pose)
bproc.renderer.set_max_amount_of_samples(args.samples)

light = bproc.types.Light()
light.set_type("POINT")
light.set_location((2.5 * cam_pose[:3, -1]).tolist())
light.set_energy(1000)

os.makedirs(args.output_dir, exist_ok=True)
data = bproc.renderer.render()
color_before = data["colors"][0].copy()
cv2.imwrite(
    os.path.join(args.output_dir, "color_before_nocs.png"),
    color_before[..., :3][..., ::-1],
)
data.update(bproc.renderer.render_nocs())
color_after_live = data["colors"][0]
cv2.imwrite(
    os.path.join(args.output_dir, "color_after_nocs_from_original_data.png"),
    color_before[..., :3][..., ::-1],
)
cv2.imwrite(
    os.path.join(args.output_dir, "color_after_nocs_live_data.png"),
    color_after_live[..., :3][..., ::-1],
)
cv2.imwrite(
    os.path.join(args.output_dir, "nocs_alpha_mask.png"),
    (data["nocs"][0][..., -1] * 255).astype(np.uint8),
)

rgb = color_before[..., :3]
mask = data["nocs"][0][..., -1] > 0
obj_rgb = rgb[mask]
print(f"RGB_UNIQUE_ALL={np.unique(rgb.reshape(-1, 3), axis=0).shape[0]}")
print(f"RGB_UNIQUE_OBJ={np.unique(obj_rgb, axis=0).shape[0] if len(obj_rgb) else 0}")
print(f"RGB_MEAN_OBJ={obj_rgb.mean(axis=0).tolist() if len(obj_rgb) else None}")
print(f"RGB_STD_OBJ={obj_rgb.std(axis=0).tolist() if len(obj_rgb) else None}")
live_rgb = color_after_live[..., :3]
live_obj_rgb = live_rgb[mask]
print(f"LIVE_RGB_UNIQUE_OBJ={np.unique(live_obj_rgb, axis=0).shape[0] if len(live_obj_rgb) else 0}")
print(f"LIVE_RGB_MEAN_OBJ={live_obj_rgb.mean(axis=0).tolist() if len(live_obj_rgb) else None}")
print(f"LIVE_RGB_STD_OBJ={live_obj_rgb.std(axis=0).tolist() if len(live_obj_rgb) else None}")
