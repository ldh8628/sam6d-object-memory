import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('publish',ROOT/'run_orb3_publish.py')
publish=importlib.util.module_from_spec(spec); spec.loader.exec_module(publish)
spec=importlib.util.spec_from_file_location('prepare',ROOT/'orb3_publish/prepare_tum.py')
prepare=importlib.util.module_from_spec(spec); spec.loader.exec_module(prepare)

class ConfigurationTests(unittest.TestCase):
    def args(self,*extra):
        return publish.parser().parse_args(['--atlas','map.osa','--settings','camera.yaml',*extra])
    def test_invalid_limits_rejected(self):
        for extra in [('--queue-capacity','0'),('--queue-capacity','-1'),('--max-frames','-1'),
                      ('--replay-rate','nan'),('--color-exposure','inf'),('--domain-id','233'),
                      ('--discovery-server','localhost'),('--discovery-server','10.77.0.2:99999')]:
            with self.subTest(extra=extra),self.assertRaises(ValueError): publish.validate_args(self.args(*extra))
    def test_default_and_discovery(self):
        publish.validate_args(self.args())
        publish.validate_args(self.args('--discovery-server','10.77.0.2:11811'))
    def test_opencv_round_trip_and_no_calibration_change(self):
        import cv2
        source=ROOT/'orb3_publish/config/TUM1.yaml'
        cfg=publish.load_settings(source)
        cfg['System.LoadAtlasFromFile']='../한글 지도/map'
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'settings.yaml'; publish.write_opencv_settings(path,cfg)
            fs=cv2.FileStorage(str(path),cv2.FILE_STORAGE_READ)
            self.assertTrue(fs.isOpened())
            self.assertAlmostEqual(fs.getNode('Camera1.fx').real(),cfg['Camera1.fx'])
            self.assertEqual(fs.getNode('File.version').string(),'1.0')
            self.assertEqual(fs.getNode('System.LoadAtlasFromFile').string(),cfg['System.LoadAtlasFromFile'])
            fs.release()
            self.assertEqual(publish.load_settings(path),cfg)
    def test_bad_settings_do_not_reach_native(self):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'settings.yaml'
            for text in ['%YAML:1.0\n---\n[]\n', '%YAML:1.0\n---\nFile.version: 1.0\nCamera.type: FishEye\n']:
                path.write_text(text)
                with self.assertRaises(ValueError): publish.load_settings(path)
    def test_association_never_reuses_depth(self):
        matches,rgb,depth=prepare.associate({1:'a',1.01:'b',2:'c'},{1.009:'d',2.01:'e'})
        self.assertEqual(matches,[(1.01,1.009),(2,2.01)])
        self.assertEqual(rgb,[1]); self.assertEqual(depth,[])
    def test_replay_geometry_matches_calibration(self):
        import cv2
        import numpy as np
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)
            cv2.imwrite(str(path/'rgb.png'),np.zeros((20,30,3),dtype=np.uint8))
            cv2.imwrite(str(path/'depth.png'),np.ones((20,30),dtype=np.uint16))
            (path/'associations.txt').write_text('# test pair\n1 rgb.png 1 depth.png\n')
            publish.validate_replay_geometry(path,{'Camera.width':30,'Camera.height':20})
            with self.assertRaises(ValueError):
                publish.validate_replay_geometry(path,{'Camera.width':640,'Camera.height':480})
    def test_accuracy_known_rigid_transform(self):
        import numpy as np
        from scipy.spatial.transform import Rotation
        spec=importlib.util.spec_from_file_location('evaluate',ROOT/'orb3_publish/evaluate.py')
        ev=importlib.util.module_from_spec(spec); spec.loader.exec_module(ev)
        times=np.arange(100)*.03
        pos=np.column_stack([np.sin(times),np.cos(times),times*.1])
        rot=Rotation.from_euler('z',times)
        transform=Rotation.from_euler('xyz',[.3,-.4,.9]); translation=np.array([1,3,4])
        est=np.column_stack([times,transform.apply(pos)+translation,(transform*rot).as_quat()])
        gt=np.column_stack([times,pos,rot.as_quat()])
        result=ev.evaluate(est,gt)
        self.assertLess(result['ate_rmse_m'],1e-10)
        self.assertLess(result['rpe_rotation_rmse_deg'],1e-10)
        self.assertLess(result['rpe_translation_rmse_m'],1e-10)
        self.assertGreater(result['fast_rotation_pairs'],90)

if __name__=='__main__': unittest.main()
