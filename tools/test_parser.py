import csv
import io
import struct
import tempfile
import unittest
import wave
from pathlib import Path
from parser import HEADER, MODULO, ClockUnwrapper, ParseError, iter_packets, parse_binary, LSM_M_S2_PER_LSB
from test_acquisition_parser import packet as high_rate_packet


def packet(sensor, timestamp, payload=b''):
    return HEADER.pack(0xaa, sensor, timestamp, len(payload)) + payload


class ParserTests(unittest.TestCase):
    def test_invalid_header_stops_or_advances(self):
        data = b'bad bytes here' + packet(255, 123)
        with self.assertRaises(ParseError):
            list(iter_packets(io.BytesIO(data)))
        stats = {}
        from collections import Counter
        stats = Counter()
        recovered = list(iter_packets(io.BytesIO(data), True, stats))
        self.assertEqual(len(recovered), 1)
        self.assertEqual(stats['resync_skipped_bytes'], 14)

    def test_truncation_and_wrong_lengths(self):
        for data in [b'xx', HEADER.pack(0xaa, 17, 0, 599), HEADER.pack(0xaa, 17, 0, 628) + b'abc']:
            with self.assertRaises(ParseError):
                list(iter_packets(io.BytesIO(data)))

    def test_rollover_and_backward_time(self):
        clock = ClockUnwrapper()
        self.assertEqual(clock.unwrap(1, MODULO - 100), MODULO - 100)
        self.assertEqual(clock.unwrap(1, 100), MODULO + 100)
        self.assertEqual(clock.unwrap(2, 110), MODULO + 110)
        with self.assertRaises(ParseError):
            clock.unwrap(1, 99)

    def test_current_acquisition_bme_and_audio(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            adxl = struct.pack('<300h', *([-2, 0, 10] * 100))
            audio = struct.pack('<256h', *([1] * 128 + [-2] * 128))
            data = high_rate_packet(17, 100000, count=100, irq=100000, data=adxl)
            data += packet(5, 161000, struct.pack('<fff', 25, 98000, 50))
            data += packet(7, 162000, audio) + packet(255, 163000)
            source = root / 'input.bin'; source.write_bytes(data)
            summary = parse_binary(source)
            out = root / 'input_parsed'
            self.assertEqual(summary['packet_counts']['adxl371_main'], 1)
            self.assertEqual(summary['termination']['type'], 'EOF')
            with (out / 'adxl371_main.csv').open() as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), 100)
            self.assertAlmostEqual(float(rows[99]['timestamp_us']), 100000)
            with wave.open(str(out / 'flight_audio.wav')) as w:
                self.assertEqual(w.getnframes(), 128)
                self.assertEqual(struct.unpack('<hh', w.readframes(1)), (1, -2))
            with self.assertRaises(FileExistsError):
                parse_binary(source)


if __name__ == '__main__':
    unittest.main()
