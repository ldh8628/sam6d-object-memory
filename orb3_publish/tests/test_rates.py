import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('publish',ROOT/'run_orb3_publish.py')
publish=importlib.util.module_from_spec(spec);spec.loader.exec_module(publish)

class FrameRates(unittest.TestCase):
    def check_rate(self,fps,ingress_fps=None):
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'frames.csv'
            with path.open('w') as f:
                writer=csv.writer(f);writer.writerow(['capture_ns','enqueue_ns','tracking','enqueue_to_publish_ms'])
                for i in range(31):writer.writerow([int(i*1e9/fps),int(i*1e9/(ingress_fps or fps))+10000000,int(i>=2),50 if i==5 else 10])
            return publish.frame_rate_report(path,30)
    def test_regular_30hz_and_invalid_frames_are_counted(self):
        report=self.check_rate(30)
        self.assertTrue(report['source_rate_verified'])
        self.assertTrue(report['ingress_rate_verified'])
        self.assertAlmostEqual(report['observed_capture_hz'],30)
        self.assertEqual(report['first_valid_sequence'],3)
        self.assertEqual(report['longest_invalid_run_frames'],2)
        self.assertEqual(report['all_frames_over_33ms'],1)
    def test_slow_sensor_does_not_pass_rate_check(self):
        report=self.check_rate(15)
        self.assertFalse(report['source_rate_verified'])
        self.assertEqual(report['capture_intervals_over_1_5_period'],30)
    def test_retimed_slow_replay_does_not_pass_ingress_rate(self):
        report=self.check_rate(30,15)
        self.assertTrue(report['source_rate_verified'])
        self.assertFalse(report['ingress_rate_verified'])
