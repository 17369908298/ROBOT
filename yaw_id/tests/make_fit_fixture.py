"""Known synthetic plant for offline MATLAB regression; this is not captured motor data."""
import csv
import importlib.util
import math
from pathlib import Path
import struct
import zlib

root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('capture',root/'tools/decode_yaw.py')
capture=importlib.util.module_from_spec(spec);spec.loader.exec_module(capture)
path=root/'build/host_tests/synthetic.csv';path.parent.mkdir(parents=True,exist_ok=True)
legacy_path=path.with_name('synthetic_schema3.csv')
schema4_path=path.with_name('synthetic_schema4.csv')

def physical_values(t):
    speed=math.sin(2*math.pi*.3*t)+.2*math.sin(2*math.pi*2*t)
    accel=2*math.pi*.3*math.cos(2*math.pi*.3*t)+.4*math.pi*2*math.cos(2*math.pi*2*t)
    torque=.012*accel+.02*speed+.06*math.tanh(speed/.1)
    return speed,torque

def write_frame(writer,values):
    body=struct.pack('<'+str(len(values))+'f',*values)
    writer.writerow(struct.unpack('<'+str(len(values)+1)+'f',body+struct.pack('<f',zlib.crc32(body)&0xffffff)))

with path.open('w',newline='',encoding='utf-8') as stream, legacy_path.open('w',newline='',encoding='utf-8') as legacy, schema4_path.open('w',newline='',encoding='utf-8') as old_compact:
    writer=csv.writer(stream);writer.writerow(capture.COLUMNS)
    legacy_writer=csv.writer(legacy);legacy_writer.writerow(capture.COLUMNS_V3)
    schema4_writer=csv.writer(old_compact);schema4_writer.writerow(capture.COLUMNS_V4)
    for index in range(10000):
        t=index*.005
        speed,torque=physical_values(t)
        write_frame(schema4_writer,[t,torque,0,speed,1,0,index,2,4])
        values=[t,torque,0,speed,0,0,1,0,0,2,index,index,index,0,0,0,0,1,.3,0,25,0,3]
        write_frame(legacy_writer,values)
        # Jitter the receive spacing, and repeat some complete IMU frames.
        imu_time=t-.002-.001*math.sin(2*math.pi*.7*t)
        imu_speed,imu_torque=physical_values(imu_time)
        values=[t,imu_torque,.7,imu_speed,1,0,index*2,imu_time,5]
        write_frame(writer,values)
        if index>0 and index%100==0:
            values[0]+=.001;values[6]+=1
            write_frame(writer,values)
print(path)
