"""Run: python -m unittest discover -s integration -p test_input_integrity.py."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest

from input_integrity import SequentialInput, read_rows, summarize


def continuous():
    rows=[]
    for i in range(190):
        received=int(i*1e9/30)+1_000_000_000
        stamp=100_000_000_000+int(i*1e9/30)
        for stream,offset in [('rgb',100),('depth',700)]:
            rows.append(dict(kind='metadata',stage='monitor',camera='slam_camera',serial='123',
                stream=stream,sequence=i,received_ns=received,stamp_ns=stamp,
                metadata={'frame_counter':i+offset},
                frame_number=i+offset,sensor_time_ms=i*1000/30,hardware_counter=True,
                consumed=True,queue_ms=.1,callback_ms=.05))
        rows.append(dict(kind='rgbd',stage='monitor',camera='slam_camera',serial='123',
            sequence=i,received_ns=received,stamp_ns=stamp,depth_stamp_ns=stamp,
            consumed=True,queue_ms=.2,callback_ms=.1,hashes={'rgb':str(i),'depth':str(i)}))
    config=dict(cameras={'slam_camera':'123'},consumers={},start_ns=2_000_000_000,
        end_ns=6_000_000_000,seconds=4,window_closed=True,drained=True,qos_verified=True,
        is_baseline=True,record=False)
    return rows,config


class IntegrityChecks(unittest.TestCase):
    def test_automatic_camera_baseline_is_required_before_load(self):
        from unittest.mock import Mock, patch
        from input_check import ensure_camera_baseline
        cameras = {'slam_camera':'123','sam_camera':'456'}
        with patch('input_check.InputCheckSession') as factory:
            existing = Path('/existing/input_integrity.json')
            self.assertEqual(ensure_camera_baseline(existing, '/run', 73, cameras,
                [], 30, settings={}), existing)
            factory.assert_not_called()
        for status in ('PASS','FAIL','INCOMPLETE'):
            with self.subTest(status=status):
                session = Mock()
                session.done.side_effect = [False,True]
                session.finish.return_value = {'status':status}
                with patch('input_check.InputCheckSession', return_value=session) as factory, \
                        patch('input_check.time.sleep'):
                    if status == 'PASS':
                        self.assertEqual(ensure_camera_baseline(None, '/run', 73, cameras,
                            [], 30, settings={'hardware_sync':False}),
                            Path('/run/input_baseline/input_integrity.json'))
                    else:
                        with self.assertRaisesRegex(RuntimeError, status):
                            ensure_camera_baseline(None, '/run', 73, cameras,
                                [], 30, settings={'hardware_sync':False})
                factory.assert_called_once_with(Path('/run/input_baseline'), 73, cameras,
                    seconds=30, is_baseline=True, settings={'hardware_sync':False})
                session.wait_ready.assert_called_once_with([],30)
                session.arm.assert_called_once()
                session.close_window.assert_called_once()
                session.finish.assert_called_once_with(drained=True,error=None,keep_publishers=True)

    def test_continuous_independent_sensor_numbers(self):
        rows,cfg=continuous(); result=summarize(rows,cfg)
        self.assertEqual(result['status'],'PASS',result)
        live=next(r for r in rows if r['kind']=='rgbd' and 'identity' in r)
        self.assertNotEqual(live['identity']['rgb']['number'],live['identity']['depth']['number'])
        self.assertEqual(result['cameras']['slam_camera']['received_rgbd'],120)

    def test_gap_duplicate_reverse_restart_missing_metadata(self):
        for mode in ('gap','duplicate','reverse','restart','missing'):
            with self.subTest(mode=mode):
                rows,cfg=continuous()
                metadata=[r for r in rows if r.get('stream')=='rgb']
                if mode=='gap': metadata[80]['frame_number']+=1
                if mode=='duplicate': metadata[80]['frame_number']=metadata[79]['frame_number']
                if mode=='reverse': metadata[80]['frame_number']=metadata[78]['frame_number']
                if mode=='restart':
                    for i,r in enumerate(metadata[80:]): r['frame_number']=i
                if mode=='missing':
                    for r in metadata: r.pop('frame_number')
                self.assertNotEqual(summarize(rows,cfg)['status'],'PASS')

    def test_independent_depth_window_offsets(self):
        for offset in (-1_000_000, 1_000_000):
            rows,cfg=continuous()
            for row in rows:
                if row.get('stream')=='depth': row['stamp_ns']+=offset
                if row['kind']=='rgbd': row['depth_stamp_ns']+=offset
            self.assertEqual(summarize(rows,cfg)['status'],'PASS')

    def test_stalls_at_window_boundaries_cannot_pass(self):
        for mode in ('start','burst'):
            rows,cfg=continuous()
            for row in rows:
                if mode=='start' and row['sequence']>=30:
                    row['received_ns']+=3_000_000_000
                elif mode=='burst' and 30<=row['sequence']<150:
                    row['received_ns']=cfg['start_ns']+(row['sequence']-30)*100_000
            self.assertEqual(summarize(rows,cfg)['status'],'FAIL')

    def test_metadata_gaps_crossing_window_boundaries(self):
        for offset in (-1_000_000, 0, 1_000_000):
            for sequence in (20, 30, 149, 170):
                with self.subTest(depth_offset=offset, missing_sequence=sequence):
                    rows,cfg=continuous()
                    rows=[r for r in rows if r['sequence'] != sequence]
                    for row in rows:
                        if row.get('stream')=='depth': row['stamp_ns']+=offset
                        if row['kind']=='rgbd': row['depth_stamp_ns']+=offset
                    result=summarize(rows,cfg)
                    measured=sequence in (30,149)
                    self.assertEqual(result['status'],'FAIL' if measured else 'PASS',result)
                    if measured:
                        gaps=[r for r in result['failures'] if r['event']=='sensor_metadata_gap']
                        self.assertEqual({r['stream'] for r in gaps},{'rgb','depth'})
                        self.assertEqual({tuple(r['missing_range']) for r in gaps},
                                         {(sequence+100,sequence+100),(sequence+700,sequence+700)})

    def test_simultaneous_stamp_and_counter_reset(self):
        rows,cfg=continuous()
        for row in rows:
            if row['sequence']>=80:
                row['stamp_ns']-=100_000_000_000
                if 'depth_stamp_ns' in row: row['depth_stamp_ns']-=100_000_000_000
                if 'frame_number' in row: row['frame_number']-=80
                if 'sensor_time_ms' in row: row['sensor_time_ms']-=80*1000/30
        result=summarize(rows,cfg)
        self.assertEqual(result['status'],'FAIL')
        self.assertTrue(any(r['event']=='restart_or_out_of_order' for r in result['failures']))

    def test_wrong_serial_loaded_baseline_and_sync_mismatch(self):
        rows,cfg=continuous();base=summarize(deepcopy(rows),deepcopy(cfg))
        for mode in ('serial','loaded','sync'):
            sample,config,baseline=deepcopy(rows),deepcopy(cfg),deepcopy(base)
            if mode=='serial': sample[2]['serial']='wrong'
            if mode=='loaded': baseline['config']['consumers']={'slam_camera':['orb']}
            if mode=='sync': baseline['config']['settings']={'hardware_sync':True}
            self.assertNotEqual(summarize(sample,config,baseline=baseline)['status'],'PASS')

    def test_partial_log_still_produces_incomplete_report(self):
        from input_check import finalize
        rows,cfg=continuous()
        with tempfile.TemporaryDirectory() as directory:
            output=Path(directory)
            path=output/'input_check_config.json';path.write_text(json.dumps(cfg))
            (output/'monitor_state.json').write_text(json.dumps({'monitor_drained':True}))
            (output/'slam_camera_rgbd_health.jsonl').write_text(
                ''.join(json.dumps(r)+'\n' for r in rows)+'{"partial":')
            result=finalize(path)
            self.assertEqual(result['status'],'INCOMPLETE')
            self.assertTrue((output/'input_integrity.json').exists())

    def test_queue_delay_and_sustained_growth(self):
        rows,cfg=continuous()
        frames=[r for r in rows if r['kind']=='rgbd']
        for i,row in enumerate(frames): row['queue_ms']=i
        self.assertEqual(summarize(rows,cfg)['status'],'FAIL')

    def test_recording_tail_content_order_and_delayed_bag(self):
        rows,cfg=continuous(); cfg['record']=True
        bag=deepcopy(rows)
        for r in bag: r['recorded_ns']=10**20
        self.assertEqual(summarize(deepcopy(rows),cfg,bag_rows=bag)['status'],'PASS')
        selected=[i for i,r in enumerate(bag) if r['kind']=='rgbd' and 30<=r['sequence']<150]
        for mode in ('tail','content','order','metadata'):
            changed=deepcopy(bag)
            if mode=='tail': del changed[selected[-1]]
            elif mode=='content': changed[selected[0]]['hashes']['rgb']='corrupt'
            elif mode=='metadata':
                changed=[r for r in changed if not (r.get('stream')=='rgb' and r['sequence']==80)]
            else:
                a,b=selected[0:2];changed[a],changed[b]=changed[b],changed[a]
            result=summarize(deepcopy(rows),cfg,bag_rows=changed)
            self.assertEqual(result['status'],'FAIL')
            if mode=='metadata':
                failure=next(r for r in result['failures'] if r['event']=='recorder_metadata_mismatch')
                self.assertEqual(failure['missing'][0]['frame_number'],180)

    def test_incomplete_window_baseline_drain_and_qos(self):
        for key in ('window_closed','drained','qos_verified','is_baseline'):
            rows,cfg=continuous();cfg[key]=False
            self.assertEqual(summarize(rows,cfg)['status'],'INCOMPLETE',key)
        rows,cfg=continuous();cfg['end_ns']=3_000_000_000
        self.assertEqual(summarize(rows,cfg)['status'],'INCOMPLETE')

    def test_consumer_tail_loss(self):
        rows,cfg=continuous();cfg['consumers']={'slam_camera':['orb']}
        consumer=[dict(r,stage='orb') for r in rows if r['kind']=='rgbd']
        rows+=consumer
        self.assertEqual(summarize(deepcopy(rows),cfg)['status'],'PASS')
        consumer[149]['consumed']=False
        self.assertEqual(summarize(rows,cfg)['status'],'FAIL')

    def test_bounded_fifo_overflow_and_shutdown_tail(self):
        entered,release=threading.Event(),threading.Event()
        consumed=[]
        def consume(value,row):
            if value==0: entered.set(); release.wait(2)
            consumed.append(value)
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'health.jsonl'
            worker=SequentialInput(path,consume,lambda value,identity_only=False:dict(kind='rgbd',stamp_ns=value),
                                   stage='test',camera='test',capacity=2)
            self.assertTrue(worker.put(0));self.assertTrue(entered.wait(1))
            self.assertTrue(worker.put(1));self.assertTrue(worker.put(2))
            self.assertFalse(worker.put(3));release.set()
            self.assertFalse(worker.close())
            self.assertEqual(consumed,[0,1,2])
            records=read_rows([path])
            self.assertEqual(len(records),4)
            self.assertEqual([r['stamp_ns'] for r in records if r.get('event')=='queue_overflow'],[3])
            self.assertFalse(worker.put(4))

    def test_consumer_error_is_logged_and_following_tail_drains(self):
        def consume(value,row):
            if value==1: raise ValueError('broken frame')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'health.jsonl'
            w=SequentialInput(path,consume,lambda value,identity_only=False:dict(stamp_ns=value),stage='x',camera='x')
            for value in range(3): w.put(value)
            self.assertFalse(w.close())
            rows=read_rows([path])
            self.assertEqual([r['consumed'] for r in rows],[True,False,True])
            self.assertEqual(rows[1]['event'],'consume_error')


if __name__=='__main__': unittest.main()
