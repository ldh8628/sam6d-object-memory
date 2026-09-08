"""Offline checks for distributed configuration and fail-closed controller startup."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml
import two_host_realtime as live


def config():
    return dict(ssh=dict(target='test@host', port=10022,remote_root='/remote/project'),
        network=dict(ros_domain_id=72, local_ptp_interface='eth0',remote_ptp_interface='eth1',ptp_max_offset_us=1000),
        cameras=dict(slam_serial='123',sam_serial='456',slam_sync_mode=1,sam_sync_mode=3))


class RealtimeChecks(unittest.TestCase):
    def test_configs_need_only_exported_intrinsics_not_bags_or_atlas(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); dataset=root/'session'; (dataset/'converted').mkdir(parents=True)
            (dataset/'camera_extrinsic').mkdir()
            intrinsics=dict(width=640,height=480,K=[500,0,320,0,500,240,0,0,1],D=[0]*5)
            info={role:dict(camera=intrinsics,color_hz=30,color_topic='/rgb',depth_topic='/depth')
                  for role in ('SLAM','SAM')}
            (dataset/'converted/info.json').write_text(json.dumps(info))
            matrix=[[1,0,0,.1],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
            (dataset/'camera_extrinsic/camera_extrinsic.json').write_text(json.dumps(
                dict(dataset='session',X_slam_camera_to_sam_camera=matrix)))
            (dataset/'camera_extrinsic/camera_extrinsic.urdf').write_text('''<robot name="rig"><joint name="rig" type="fixed"><parent link="slam_camera_color_optical_frame"/><child link="sam_camera_color_optical_frame"/><origin xyz="0.1 0 0" rpy="0 0 0"/></joint></robot>''')
            output=root/'run'; output.mkdir(); remote=Path('/remote/project/output/session/object_memory/run')
            args=SimpleNamespace(map_dir=dataset,urdf=None,view=False,record=False,slam_wait_ms=1000,pose_match_ms=50)
            bundle,sam_path,atlas,_=live.write_configs(args, config(), output, remote,
                dict(remote_dataset='/remote/project/output/session'))
            orb=yaml.safe_load((bundle/'orb_config.yaml').read_text())
            sam=yaml.safe_load(sam_path.read_text())
            self.assertFalse(atlas.exists())
            self.assertFalse(list(output.rglob('*.osa')))
            self.assertEqual(orb['topics']['rgbd'],'/camera/slam_camera/rgbd')
            self.assertEqual(orb['output']['dir'],str(remote/'orbslam3'))
            self.assertTrue(orb['orbslam3']['settings_path'].startswith('/remote/project/'))
            self.assertEqual(sam['topics']['rgbd'],'/camera/sam_camera/rgbd')
            self.assertEqual(sam['slam']['two_host_guard_file'],str(output/'two_host_guard.json'))
            self.assertFalse(sam['runtime']['use_sim_time'])
            self.assertFalse(sam['output']['live_recording']['enabled'])

    def test_preflight_failure_leaves_unhealthy_guard_and_failure_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); dataset=root/'session'; dataset.mkdir()
            manifest=dict(status='PASS',synchronization=dict(status='PASS'),cameras=config()['cameras'],
                          remote_dataset='/remote/project/output/session')
            (dataset/'distributed_map.json').write_text(json.dumps(manifest))
            args=SimpleNamespace(map_dir=dataset,two_host_config=root/'config',output=root/'run')
            with patch.object(live,'load_config',return_value=config()), \
                 patch.object(live,'preflight',side_effect=RuntimeError('PTP unavailable')), \
                 patch.object(live,'RemoteSession') as session:
                with self.assertRaisesRegex(RuntimeError,'PTP unavailable'):
                    live.run(args)
                session.assert_not_called()
            guard=json.loads((root/'run/two_host_guard.json').read_text())
            report=json.loads((root/'run/two_host_report.json').read_text())
            self.assertFalse(guard['healthy'])
            self.assertEqual(report['status'],'FAIL')
            self.assertEqual(report['orb']['consumed'],0)

    def test_acceptance_requires_measured_pose_and_quality(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp)
            report = dict(status='PASS', failures=[])
            live.finalize_report(output, report)
            self.assertEqual(report['status'], 'INCOMPLETE')
            (output/'bridge').mkdir()
            (output/'bridge/two_host_bridge_metrics.json').write_text(json.dumps(dict(
                tracking_ratio=0, capture_to_arrival_ms_p95=None)))
            report = dict(status='PASS', failures=[])
            live.finalize_report(output, report)
            self.assertEqual(report['status'], 'FAIL')
            (output/'bridge/two_host_bridge_metrics.json').write_text(json.dumps(dict(
                tracking_ratio=.99, capture_to_arrival_ms_p95=25)))
            (output/'sam_metrics.json').write_text(json.dumps(dict(
                pose_matching=dict(tracking_exact=99,tracking_missed=1),
                object_memory_quality=dict(gate_passed=True))))
            report = dict(status='PASS', failures=[])
            live.finalize_report(output, report)
            self.assertEqual(report['acceptance']['status'], 'PASS')

    def test_wrong_camera_contract_never_starts_hardware(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'distributed_map.json').write_text(json.dumps(dict(status='PASS',
                synchronization=dict(status='PASS'),cameras={})))
            with patch.object(live,'load_config',return_value=config()), \
                 patch.object(live,'RemoteSession') as session:
                with self.assertRaisesRegex(ValueError,'camera roles'):
                    live.run(SimpleNamespace(map_dir=root,two_host_config=root/'config'))
                session.assert_not_called()

if __name__=='__main__':
    unittest.main()
