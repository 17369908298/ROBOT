"""Decode an existing JustFloat binary file offline. No serial port is opened.

python tools/decode_yaw.py --decode data/run.bin --out data/replay.csv
Live recording is performed directly by VOFA, not by this utility.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct
import time
import zlib

COLUMNS_V3 = (
    'time_s tau_cmd_nm theta_rad omega_rad_s tau_base_nm tau_id_nm mode flags '
    'motor_feedback_age_ms imu_age_ms frame_sequence motor_sequence imu_sequence '
    'tx_drops can_submit_age_ms tau_meas_nm can_submit_torque_nm '
    'excitation_frequency_hz excitation_amplitude_nm excitation_stage '
    'motor_temperature_c pitch_rad schema_version frame_crc24'
).split()
COLUMNS_V4 = (
    'time_s tau_cmd_nm theta_rad omega_rad_s mode flags frame_sequence '
    'feedback_age_ms schema_version frame_crc24'
).split()
COLUMNS = (
    'time_s tau_cmd_nm theta_rad omega_rad_s mode flags frame_sequence '
    'imu_time_s schema_version frame_crc24'
).split()
SCHEMAS = {3: COLUMNS_V3, 4: COLUMNS_V4, 5: COLUMNS}
TAIL = b'\x00\x00\x80\x7f'
FRAME_BYTES = len(COLUMNS)*4+4


class Decoder:
    def __init__(self, schema=5):
        self.schema = schema
        self.columns = SCHEMAS[schema]
        self.payload_bytes = len(self.columns)*4
        self.frame_bytes = self.payload_bytes+len(TAIL)
        self.unpack_format = '<'+str(len(self.columns))+'f'
        self.buffer = bytearray()
        self.valid_frames = self.rejected_candidates = self.discarded_bytes = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= self.frame_bytes:
            end = self.buffer.find(TAIL, self.payload_bytes)
            if end < 0:
                count = len(self.buffer) - (self.frame_bytes-1)
                del self.buffer[:count]
                self.discarded_bytes += count
                break
            start = end - self.payload_bytes
            payload = bytes(self.buffer[start:end])
            values = struct.unpack(self.unpack_format, payload)
            valid = (all(map(math.isfinite, values)) and values[-2] == float(self.schema) and
                     values[-1] == float(zlib.crc32(payload[:-4]) & 0xffffff))
            if valid:
                frames.append(values)
                self.valid_frames += 1
                self.discarded_bytes += start
                del self.buffer[:end+4]
            else:
                self.rejected_candidates += 1
                # Scan every possible tail alignment; a payload can contain the tail bytes.
                count = start + 1
                del self.buffer[:count]
                self.discarded_bytes += count
        return frames


class Statistics:
    def __init__(self, columns=COLUMNS):
        self.index = {name: index for index, name in enumerate(columns)}
        self.last_sequence = self.last_time = None
        self.missing_sequence_frames = self.duplicates = self.rounds = 0
        self.active_frames = 0
        self.dt_min = self.dt_max = None
        self.dt_sum = self.dt_count = 0
        self.max_feedback_age_ms = 0
        self.last_imu_time = None
        self.imu_repeated_frames = self.imu_timestamp_regressions = 0
        self.max_tx_drops = 0 if 'tx_drops' in self.index else None
        self.flag_or = 0

    def add(self, row):
        seq = int(row[self.index['frame_sequence']])
        if self.last_sequence is not None:
            delta = (seq-self.last_sequence) & 0xffffff
            if delta == 0:
                self.duplicates += 1
            elif delta < 0x800000:
                self.missing_sequence_frames += delta-1
            else:
                self.last_time = None  # reset/reboot, not millions of missing packets
        self.last_sequence = seq
        if row[self.index['mode']] > 0:
            self.active_frames += 1
            if 'imu_time_s' in self.index:
                imu_time = row[self.index['imu_time_s']]
                age = max(0, (row[0]-imu_time)*1000)
                if self.last_time is not None and row[0]>self.last_time and self.last_imu_time is not None:
                    self.imu_repeated_frames += int(imu_time==self.last_imu_time)
                    self.imu_timestamp_regressions += int(imu_time<self.last_imu_time)
                self.last_imu_time = imu_time
            elif 'feedback_age_ms' in self.index:
                age = row[self.index['feedback_age_ms']]
            else:
                age = max(row[self.index['motor_feedback_age_ms']], row[self.index['imu_age_ms']])
            self.max_feedback_age_ms = max(self.max_feedback_age_ms, age)
            if self.last_time is None or row[0] < self.last_time:
                self.rounds += 1
            elif row[0] > self.last_time:
                dt = row[0]-self.last_time
                self.dt_min = dt if self.dt_min is None else min(self.dt_min, dt)
                self.dt_max = dt if self.dt_max is None else max(self.dt_max, dt)
                self.dt_sum += dt
                self.dt_count += 1
            self.last_time = row[0]
        else:
            self.last_time = None
            self.last_imu_time = None
        if self.max_tx_drops is not None:
            self.max_tx_drops = max(self.max_tx_drops, int(row[self.index['tx_drops']]))
        self.flag_or |= int(row[self.index['flags']])

    def report(self):
        result = vars(self).copy()
        result.pop('index')
        result['mean_active_interval_ms'] = 1000*self.dt_sum/self.dt_count if self.dt_count else None
        result['min_active_interval_ms'] = 1000*self.dt_min if self.dt_min is not None else None
        result['max_active_interval_ms'] = 1000*self.dt_max if self.dt_max is not None else None
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--decode', type=Path, required=True)
    parser.add_argument('--schema', type=int, choices=SCHEMAS, default=5,
                        help='5: IMU timestamps; 4: old compact; 3: old 24-channel capture')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--notes', default='')
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    meta_path = args.out.with_suffix('.meta.json')
    if args.decode.resolve() == args.out.resolve():
        parser.error('input and output must differ')
    # Refuse accidental overwrites of previous experiments.
    for path in [args.out, meta_path]:
        if path.exists():
            parser.error('Output already exists: '+str(path))
    decoder = Decoder(args.schema)
    stats = Statistics(decoder.columns)
    start = time.monotonic()
    source_stream = None
    byte_count = 0
    error = None
    try:
        source_stream = args.decode.open('rb')
        with args.out.open('x', newline='', encoding='utf-8') as output:
            writer = csv.writer(output)
            writer.writerow(decoder.columns)
            while True:
                chunk = source_stream.read(4096)
                if not chunk:
                    break
                byte_count += len(chunk)
                for row in decoder.feed(chunk):
                    writer.writerow(row)
                    stats.add(row)
    except KeyboardInterrupt:
        error = 'Interrupted; partial offline decode retained'
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        if source_stream:
            source_stream.close()
        report = dict(schema_version=args.schema, source=str(args.decode), offline_only=True,
                      elapsed_host_s=time.monotonic()-start, bytes=byte_count, notes=args.notes,
                      error=error, valid_frames=decoder.valid_frames,
                      rejected_candidates=decoder.rejected_candidates,
                      discarded_bytes=decoder.discarded_bytes, trailing_bytes=len(decoder.buffer),
                      **stats.report())
        meta_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
        print(json.dumps(report, indent=2, ensure_ascii=False))
    if not decoder.valid_frames:
        raise SystemExit(f'No valid schema-{args.schema} frames in input file: check data format/version.')


if __name__ == '__main__':
    main()
