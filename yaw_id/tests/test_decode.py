import csv
import importlib.util
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('capture',ROOT/'tools/decode_yaw.py')
capture=importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def packet(seq=1, schema=5):
    columns=capture.SCHEMAS.get(schema,capture.COLUMNS)
    v=[0.0]*(len(columns)-1)
    v[0]=seq*.005;v[columns.index('mode')]=4
    v[columns.index('frame_sequence')]=seq;v[-1]=schema
    if 'imu_time_s' in columns:v[columns.index('imu_time_s')]=seq*.005-.002
    payload=struct.pack('<'+str(len(v))+'f',*v)
    return payload+struct.pack('<f',zlib.crc32(payload)&0xffffff)+capture.TAIL


class DecodeTests(unittest.TestCase):
    def test_split_noise_corruption_resynchronization(self):
        bad=bytearray(packet(4));bad[12]^=1
        stream=b'noise'+packet(1)+packet(2)+bad+packet(3,2)+packet(5)+packet(6)[:22]
        decoder=capture.Decoder();rows=[];rng=random.Random(100)
        while stream:
            n=rng.randrange(1,94);rows+=decoder.feed(stream[:n]);stream=stream[n:]
        self.assertEqual([int(r[6]) for r in rows],[1,2,5])
        self.assertGreaterEqual(decoder.rejected_candidates,2)

    def test_trailing_payload_tail(self):
        # An infinity elsewhere in a candidate is rejected even with a valid CRC.
        v=[0.0]*9;v[3]=float('inf');v[8]=5
        body=struct.pack('<9f',*v)
        stream=body+struct.pack('<f',zlib.crc32(body)&0xffffff)+capture.TAIL+packet(1)
        self.assertEqual(len(capture.Decoder().feed(stream)),1)

    def test_replay_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            raw=Path(tmp)/'source.bin';raw.write_bytes(packet(1)+packet(2)+packet(4))
            out=Path(tmp)/'test.csv'
            run=subprocess.run([sys.executable,str(ROOT/'tools/decode_yaw.py'),'--decode',str(raw),
                                '--out',str(out)],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            with out.open(newline='') as file:rows=list(csv.DictReader(file))
            self.assertEqual(len(rows),3)
            self.assertEqual(len(rows[0]),10)
            self.assertEqual(float(rows[2]['frame_sequence']),4)
            repeated=subprocess.run([sys.executable,str(ROOT/'tools/decode_yaw.py'),'--decode',str(raw),
                                     '--out',str(out)],capture_output=True)
            self.assertNotEqual(repeated.returncode,0)
            self.assertEqual(raw.read_bytes(),packet(1)+packet(2)+packet(4))

    def test_serial_capture_removed(self):
        self.assertFalse((ROOT/'tools/capture_yaw.py').exists())
        result=subprocess.run([sys.executable,str(ROOT/'tools/decode_yaw.py'),'--port','COM16',
                               '--out','unused.csv'],capture_output=True)
        self.assertNotEqual(result.returncode,0)

    def test_statistics_wrap(self):
        stats=capture.Statistics()
        for seq in [0xfffffe,0xffffff,0,2]:
            row=list(struct.unpack('<10f',packet(seq)[:40]));stats.add(row)
        self.assertEqual(stats.missing_sequence_frames,1)

    def test_legacy_schema3_explicit(self):
        decoder=capture.Decoder(3)
        rows=decoder.feed(packet(7,3)+packet(8,3))
        self.assertEqual([int(row[10]) for row in rows],[7,8])
        self.assertEqual(len(rows[0]),24)
        self.assertEqual(capture.Decoder().feed(packet(7,3)),[])

    def test_legacy_schema4_explicit(self):
        rows=capture.Decoder(4).feed(packet(7,4))
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0][8],4)
        self.assertEqual(capture.Decoder().feed(packet(7,4)),[])

    def test_imu_receive_stamp(self):
        stats=capture.Statistics()
        rows=capture.Decoder().feed(packet(10)+packet(11))
        stats.add(rows[0]);stats.add(rows[1])
        repeated=list(rows[1]);repeated[0]+=.001
        stats.add(repeated)
        self.assertEqual(stats.imu_repeated_frames,1)
        self.assertAlmostEqual(stats.max_feedback_age_ms,3,places=4)


if __name__=='__main__':
    unittest.main()
