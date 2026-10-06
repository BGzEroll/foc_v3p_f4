#!/usr/bin/env python3
"""MCU-only reset checks with the encoder continuously powered. Motor moves
during 3 V alignment; every reset ends with a verified hardware output stop.
Run after flashing the matching image; no flash or power-cycle is performed.
"""
import argparse, json, pathlib, re, socket, struct, subprocess, time
from check_image import check_image
p=argparse.ArgumentParser();p.add_argument('--elf',default='build/PLLRelease/project.elf')
p.add_argument('--output',default='build/warm_reset.json');a=p.parse_args()
fields={'rotor.sequence':'I','rotor.error_count':'I','rotor.config_before':'I',
    'rotor.config_after':'I','rotor.magnet_status':'I','rotor.magnet_agc':'I',
    'rotor.magnet_magnitude':'I','i2c_bus_debug[0].recovery_attempts':'I',
    'i2c_bus_debug[0].recovery_failures':'I','motor_instance.state_':'B',
    'Sguan.status':'B','pll_experiment.request':'I',
    'pll_experiment.startup_encoder_delta_rad':'f','pll_experiment.startup_reverse_delta_rad':'f',
    'pll_experiment.startup_zero_spread_rad':'f','pll_experiment.startup_max_phase_a':'f'}
gdb=['gdb-multiarch','-q','-batch',a.elf]
for f in fields:gdb+=['-ex',f'p/x &{f}']
addresses=re.findall(r'= (0x[0-9a-f]+)',subprocess.check_output(gdb,text=True))
if len(addresses)!=len(fields):raise RuntimeError('Missing diagnostic symbol')
addr=dict(zip(fields,[int(x,16) for x in addresses]));s=socket.create_connection(('localhost',6666),timeout=10)
def tcl(c):
    s.sendall(c.encode()+b'\x1a');b=b''
    while not b.endswith(b'\x1a'):
        q=s.recv(65536)
        if not q:raise RuntimeError('OpenOCD disconnected')
        b+=q
    return b[:-1].decode()
def off():
    tcl('write_memory 0x40021018 32 {2097152}');tcl('mmw 0x40010444 0 0x8000')
def hardware():
    return {'MOTOR_EN':bool(int(tcl('read_memory 0x40021014 32 1').strip(),0)&32),
            'TIM8_MOE':bool(int(tcl('read_memory 0x40010444 32 1').strip(),0)&32768)}
check_image(tcl,a.elf);records=[]
try:
    for index in range(3):
        off();tcl('reset run');time.sleep(7)
        row={'reset_index':index+1}
        for f,fmt in fields.items():
            size=1 if fmt=='B' else 4
            v=int(tcl(f'read_memory {addr[f]} {size*8} 1').strip(),0)
            row[f]=struct.unpack('<'+fmt,v.to_bytes(size,'little'))[0]
        row['before_stop']=hardware()
        tcl(f'write_memory {addr["pll_experiment.request"]} 32 {{2}}');time.sleep(.1)
        off();row['after_stop']=hardware();records.append(row)
finally:
    off();s.close();path=pathlib.Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(records,indent=2)+'\n')
print(json.dumps(records,indent=2))
assert len(records)==3 and all(r['rotor.sequence']>1000 for r in records),'Encoder did not resume on every reset'
assert all(not r['after_stop']['MOTOR_EN'] and not r['after_stop']['TIM8_MOE'] for r in records)
print('PASS: encoder live after three MCU-only resets, all outputs stopped. Inspect alignment faults separately.')
