import csv
import struct
import tempfile
import unittest
from pathlib import Path
from lsm_timing import TimingEstimator
from parser import HEADER, MODULO, parse_binary
from test_acquisition_parser import packet


class TimingTests(unittest.TestCase):
    def test_fifo_backlog_correction_and_independent_epochs(self):
        est = TimingEstimator()
        for sid, epoch, rate in [(19,0,6216.6),(19,1,6217.2),(20,0,6221.5)]:
            for n in range(400):
                index = n*150
                entries = 300+2*(n%70)
                time = 1e6+epoch*20e6+(index+entries//2-1)*1e6/rate
                est.add(sid,epoch,time,index,index,entries,300)
        models, report = est.finish(6670)
        self.assertEqual(len(models),3)
        self.assertTrue(all(r['accepted'] for r in report))
        for key, rate in [((19,0),6216.6),((19,1),6217.2),((20,0),6221.5)]:
            self.assertAlmostEqual(models[key]['accel']['rate_hz'],rate,places=7)

    def test_reject_short_noisy_and_backward_epochs(self):
        est=TimingEstimator()
        for n in range(10): est.add(19,0,n*24000,n*150,n*150,300,300)
        for n in range(400):
            index=n*150
            time=1e6+(index+149)*1e6/6216.6+(10000 if n%2 else -10000)
            est.add(19,1,time,index,index,300,300)
            est.add(20,0,time,index,index,300,300)
        est.add(20,0,time+25000,1,1,300,300)
        models, report=est.finish(6670)
        self.assertFalse(models)
        self.assertIn('5 seconds',report[0]['accel']['reason'])
        self.assertIn('2 ms',report[1]['accel']['reason'])
        self.assertIn('nonmonotonic',report[2]['accel']['reason'])

    def make_recording(self, root):
        # Complete streams straddle micros rollover; retain a reset gap.
        chunks=[]
        origin=MODULO-3_000_000
        seq={19:0,20:0}
        for epoch in (0,1):
            for n in range(240):
                first=n*150
                for sid,rate in [(19,6216.6),(20,6221.5)]:
                    ts=round(origin+epoch*8e6+(first+149)*1e6/rate)
                    data=b''.join(struct.pack('<BhhhBhhh',8|((i%4)<<1),1,2,3,
                                             16|((i%4)<<1),4,5,6)
                                  for i in range(first,first+150))
                    chunks.append((ts,packet(sid,ts%MODULO,seq=seq[sid],epoch=epoch,
                                             acc=first,gyro=first,count=300,flags=0,data=data)))
                    seq[sid]+=1
        chunks.sort(key=lambda p:p[0])
        source=root/'test.bin'
        source.write_bytes(b''.join(p for _,p in chunks)+HEADER.pack(0xaa,255,(chunks[-1][0]+1)%MODULO,0))
        return source

    def test_full_parse_fit_rollover_epochs_and_raw_data(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=self.make_recording(root)
            summary=parse_binary(source)
            self.assertEqual(summary['status'],'terminated')
            self.assertEqual(summary['acquisition']['records']['lsm_main'],144000)
            self.assertEqual(summary['acquisition']['lsm_tag_step_anomalies'],{})
            self.assertEqual(summary['acquisition']['lsm_index_gaps'],{})
            self.assertEqual(len(summary['acquisition']['lsm_timing_models']),4)
            for model in summary['acquisition']['lsm_timing_models']:
                self.assertEqual(model['model'],'epoch_fifo_mcu_affine_fit')
                expected=6216.6 if model['sensor_id']==19 else 6221.5
                self.assertAlmostEqual(model['accel_rate_hz'],expected,places=3)
            with (root/'test_parsed/lsm_main_accel.csv').open() as f: rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),72000)
            self.assertTrue(all(r['raw_x']=='4' for r in rows))
            self.assertGreater(float(rows[-1]['timestamp_us']),MODULO)
            self.assertGreater(float(rows[36000]['timestamp_us'])-float(rows[35999]['timestamp_us']),2e6)
            self.assertAlmostEqual(float(rows[1]['timestamp_us'])-float(rows[0]['timestamp_us']),1e6/6216.6,places=3)

    def test_user_override_precedence_and_nominal_mode(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); source=self.make_recording(root)
            auto=parse_binary(source,root/'auto',lsm_rate_main=6200)
            models=auto['acquisition']['lsm_timing_models']
            self.assertTrue(all(m['accel_rate_hz']==6200 and 'user_rate' in m['model']
                                for m in models if m['sensor_id']==19))
            self.assertTrue(all('affine_fit' in m['model'] for m in models if m['sensor_id']==20))
            nominal=parse_binary(source,root/'nominal',lsm_timing='nominal')
            self.assertFalse(nominal['lsm_fit_report'])
            self.assertTrue(all(m['accel_rate_hz']==6670 and 'nominal' in m['model']
                                for m in nominal['acquisition']['lsm_timing_models']))


if __name__=='__main__': unittest.main()
