import csv
import struct
import tempfile
import unittest
from pathlib import Path
from parser import HEADER, ACQUISITION_META, parse_binary, ParseError


def packet(sid, ts, seq=0, epoch=0, irq=0, acc=0, gyro=0, entries=300, count=0, flags=1, data=b''):
    payload=ACQUISITION_META.pack(seq,epoch,irq,acc,gyro,entries,count,flags,0)+data
    return HEADER.pack(0xaa,sid,ts,len(payload))+payload


class AcquisitionTests(unittest.TestCase):
    def parse(self, root, data):
        p=root/'test.bin';p.write_bytes(data+HEADER.pack(0xaa,255,1000000,0))
        return parse_binary(p)

    def test_adxl_surplus_after_watermark(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)
            data=struct.pack('<330h',*([1,2,3]*110))
            self.parse(root,packet(17,110000,irq=100000,entries=330,count=110,data=data))
            with (root/'test_parsed/adxl371_main.csv').open() as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),110)
            self.assertEqual(float(rows[99]['timestamp_us']),100000)
            self.assertEqual(float(rows[109]['timestamp_us']),100000+10*195.3125)

    def test_lsm_separate_axes_missing_batch_and_epoch(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)
            pair=struct.pack('<BhhhBhhh',8,1,2,3,16,4,5,6)
            data=packet(19,101000,seq=0,irq=100000,count=2,data=pair)
            data+=packet(19,105000,seq=2,irq=104000,acc=5,gyro=5,count=2,data=pair)
            data+=packet(19,110000,seq=3,flags=6,count=0)
            data+=packet(19,130000,seq=4,epoch=1,irq=129000,count=2,data=pair)
            s=self.parse(root,data)
            self.assertEqual(s['acquisition']['missing_batch_sequences']['lsm_main'],1)
            with (root/'test_parsed/lsm_main_accel.csv').open() as f:rows=list(csv.DictReader(f))
            self.assertEqual([int(r['sample_index']) for r in rows],[0,5,0])
            self.assertAlmostEqual(float(rows[1]['timestamp_us'])-float(rows[0]['timestamp_us']),5e6/6670)
            self.assertEqual(rows[2]['epoch'],'1')
            with (root/'test_parsed/lsm_main_gyro.csv').open() as f:g=list(csv.DictReader(f))
            self.assertEqual(g[0]['raw_x'],'1');self.assertEqual(rows[0]['raw_x'],'4')

    def test_bad_count_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            with self.assertRaises(ParseError):
                self.parse(Path(t),packet(17,100000,count=2,data=struct.pack('<3h',1,2,3)))


if __name__=='__main__':unittest.main()
