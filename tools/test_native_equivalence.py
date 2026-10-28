"""Regression: native bulk extraction preserves Python metadata, numbers and WAV."""
import csv
import struct
import tempfile
import unittest
import wave
from pathlib import Path
from parser import HEADER, ACQUISITION_META, parse_binary, _native_executable
from check_run import check


def make_packet(sid, time, seq=0, epoch=0, irq=0, acc=0, gyro=0,
                entries=300, count=0, flags=0, data=b''):
    payload = ACQUISITION_META.pack(seq, epoch, irq, acc, gyro, entries, count, flags, 0) + data
    return HEADER.pack(0xaa, sid, time, len(payload)) + payload


@unittest.skipUnless(_native_executable(), 'Build native decoder to run equivalence tests')
class NativeEquivalenceTests(unittest.TestCase):
    def recording(self, root):
        pieces = []
        adxl = struct.pack('<300h', *([-100, 0, 101] * 100))
        pieces.append(make_packet(17, 110000, seq=0, irq=109000, count=100, flags=1, data=adxl))
        pieces.append(make_packet(18, 120000, seq=0, count=100, flags=0, data=adxl))
        pieces.append(make_packet(17, 135000, seq=2, irq=134000, count=100, flags=1, data=adxl))
        for seq, epoch, idx, when in [(0,0,0,145000),(1,0,2,160000),(2,0,6,175000),
                                       (3,1,0,210000)]:
            pair = struct.pack('<BhhhBhhh', 8|(seq%4)*2, 100, -200, 300,
                               16|(seq%4)*2, 10, -20, 30)
            pieces.append(make_packet(19, when, seq=seq, epoch=epoch, irq=when-80,
                acc=idx, gyro=idx, entries=100, count=2, flags=1, data=pair))
        audio = struct.pack('<256h', *([11]*128 + [-7]*128))
        pieces.append(HEADER.pack(0xaa, 7, 225000, len(audio)) + audio)
        pieces.append(HEADER.pack(0xaa, 5, 230000, 12) + struct.pack('<fff', 25, 100000, 50))
        pieces.append(HEADER.pack(0xaa, 255, 250000, 0))
        source = root/'test.bin';source.write_bytes(b''.join(pieces))
        return source

    def test_native_vs_python_and_summary_only(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);source=self.recording(root)
            py=parse_binary(source,root/'python',backend='python',lsm_timing='nominal')
            native=parse_binary(source,root/'native',backend='native',lsm_timing='nominal')
            for key in ('packet_counts','termination','status','first_packet_timestamp_us',
                        'last_packet_timestamp_us','diagnostics'):
                self.assertEqual(py[key],native[key],key)
            for key in ('records','missing_batch_sequences','events','lsm_index_gaps',
                        'lsm_tag_step_anomalies','lsm_timing_models'):
                self.assertEqual(py['acquisition'][key],native['acquisition'][key],key)
            self.assertIn('lsm_continuity',native['acquisition'])
            for name in ('adxl371_main.csv','adxl371_sat.csv','lsm_main_accel.csv',
                         'lsm_main_gyro.csv','lsm_sat_accel.csv','lsm_sat_gyro.csv',
                         'acquisition_batches.csv','bme280_main.csv','audio_packets.csv'):
                with (root/'python'/name).open(newline='') as fa, (root/'native'/name).open(newline='') as fb:
                    rows_a,rows_b=list(csv.reader(fa)),list(csv.reader(fb))
                self.assertEqual(len(rows_a),len(rows_b),name)
                for a,b in zip(rows_a,rows_b):
                    self.assertEqual(len(a),len(b),name)
                    for x,y in zip(a,b):
                        if x==y:continue
                        try:self.assertAlmostEqual(float(x),float(y),delta=1e-6)
                        except ValueError:self.fail(f'{name}: mismatch {x!r} != {y!r}')
            with wave.open(str(root/'python'/'flight_audio.wav')) as a, wave.open(str(root/'native'/'flight_audio.wav')) as b:
                self.assertEqual(a.getparams(),b.getparams())
                self.assertEqual(a.readframes(a.getnframes()),b.readframes(b.getnframes()))
            slim=parse_binary(source,root/'slim',backend='native',summary_only=True,lsm_timing='nominal')
            self.assertEqual(slim['acquisition'],native['acquisition'])
            self.assertEqual(slim['diagnostics'],native['diagnostics'])
            self.assertFalse((root/'slim'/'lsm_main_accel.csv').exists())
            self.assertFalse((root/'slim'/'flight_audio.wav').exists())
            self.assertEqual(check(root/'slim')['lsm_continuity'], check(root/'native')['lsm_continuity'])


if __name__=='__main__':unittest.main()
