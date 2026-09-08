import importlib.util
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('publish',ROOT/'run_orb3_publish.py')
publish=importlib.util.module_from_spec(spec);spec.loader.exec_module(publish)

class RosParameterTypes(unittest.TestCase):
    def test_floating_ros_parameter_defaults(self):
        args=publish.parser().parse_args(['--atlas','map.osa','--settings','camera.yaml'])
        for key in ('color_exposure','replay_rate','max_pair_skew_ms'):
            self.assertIsInstance(getattr(args,key),float,key)
    def test_incompatible_sources(self):
        args=publish.parser().parse_args(['--atlas','map.osa','--settings','camera.yaml','--dataset','a','--realsense-bag','b'])
        with self.assertRaises(ValueError):publish.validate_args(args)
    def test_recorded_d455_calibration_matches_sdk_projection(self):
        # Saved D455F calibration from this workspace, without opening a device.
        # Firmware labels it inverse Brown; verify the installed SDK numerically.
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        intr=rs.intrinsics();intr.width=640;intr.height=480
        intr.fx=384.8200988769531;intr.fy=384.2496337890625
        intr.ppx=320.78948974609375;intr.ppy=243.43768310546875
        intr.model=rs.distortion.inverse_brown_conrady
        intr.coeffs=[-.055259451270103455,.06833416223526001,-.000608497706707567,-.000944240135140717,-.02229018323123455]
        points=np.array([[(x-intr.ppx)/intr.fx,(y-intr.ppy)/intr.fy,1]
                         for y in np.linspace(0,479,13) for x in np.linspace(0,639,17)],dtype=np.float64)
        matrix=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
        projected,_=cv2.projectPoints(points,np.zeros(3),np.zeros(3),matrix,np.array(intr.coeffs))
        sdk=np.array([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in points])
        self.assertLess(np.linalg.norm(projected[:,0,:]-sdk,axis=1).max(),.5)
