#!/usr/bin/env python3
"""Bounded encoder FOC recording. Every run ends with a verified output stop.

Run from repository root after flashing the matching PLLRelease ELF. No reset
or flash is performed here. Three 256 ms recordings are taken during a 7 s run.
"""
import argparse, csv, json, pathlib, re, socket, struct, subprocess, time
from check_image import check_image

p=argparse.ArgumentParser()
p.add_argument('--elf',default='build/PLLRelease/project.elf')
p.add_argument('--output',required=True)
p.add_argument('--speed',type=float,default=40,help='Mechanical target rad/s, 5..60')
args=p.parse_args()
base=pathlib.Path(args.output);base.parent.mkdir(parents=True,exist_ok=True)
fields=['pll_experiment.direct_mode','pll_experiment.request','pll_experiment.capture_request',
    'pll_experiment.capture_count','pll_experiment.capture',
    'pll_experiment.samples','pll_experiment.fault','pll_experiment.active',
    'pll_experiment.switch_good','pll_experiment.max_loop_cycles',
    'pll_experiment.max_switch_good',
    'pll_experiment.max_period_us','pll_experiment.missed_periods',
    'pll_experiment.startup_max_phase_a','pll_experiment.startup_encoder_delta_rad',
    'Sguan.encoder.Real_Espeed','Sguan.current.Real_Id','Sguan.current.Real_Iq',
    'rotor.error_count','i2c_bus_debug[0].recovery_attempts',
    'rotor.last_read_duration_us','rotor.max_read_duration_us',
    'pll_experiment.encoder_target_speed_rad_s','pll_experiment.encoder_iq_command_a',
    'i2c_bus_debug[0].recovery_failures','Sguan.status','motor_instance.state_']
gdb=['gdb-multiarch','-q','-batch',args.elf]
for f in fields:gdb+=['-ex',f'p/x &{f}']
addresses=re.findall(r'= (0x[0-9a-f]+)',subprocess.check_output(gdb,text=True))
if len(addresses)!=len(fields):raise RuntimeError('Cannot resolve all diagnostic fields')
addr=dict(zip(fields,map(lambda a:int(a,16),addresses)))
s=socket.create_connection(('localhost',6666),timeout=10)
def tcl(c):
    s.sendall(c.encode()+b'\x1a');b=b''
    while not b.endswith(b'\x1a'):
        v=s.recv(65536)
        if not v:raise RuntimeError('OpenOCD disconnected')
        b+=v
    return b[:-1].decode()
def write(f,v):
    response=tcl(f'write_memory {addr[f]} 32 {{{v}}}')
    if response:raise RuntimeError(response)
def read(f):
    width=8 if f in ['Sguan.status','motor_instance.state_'] else 32
    n=int(tcl(f'read_memory {addr[f]} {width} 1').strip(),0)
    if f in ['pll_experiment.startup_max_phase_a','pll_experiment.startup_encoder_delta_rad',
             'Sguan.encoder.Real_Espeed','Sguan.current.Real_Id','Sguan.current.Real_Iq',
             'pll_experiment.encoder_target_speed_rad_s','pll_experiment.encoder_iq_command_a']:
        return struct.unpack('<f',n.to_bytes(4,'little'))[0]
    return n
def snapshot():
    d={f:read(f) for f in fields if f not in ['pll_experiment.capture','pll_experiment.capture_request']}
    d['MOTOR_EN']=bool(int(tcl('read_memory 0x40021014 32 1').strip(),0)&32)
    d['TIM8_MOE']=bool(int(tcl('read_memory 0x40010444 32 1').strip(),0)&32768)
    return d
check_image(tcl,args.elf)
if read('pll_experiment.direct_mode'):
    s.close()
    raise RuntimeError('Direct firmware: use run_direct_case.py instead of the encoder baseline')
if read('motor_instance.state_')!=3 or read('pll_experiment.fault'):
    raise RuntimeError('Board is not ready; reset/flash and inspect startup first')
headers='time_s,v_alpha_v,v_beta_v,i_alpha_a,i_beta_a,encoder_angle_rad,encoder_speed_rad_s,pll_angle_rad,pll_speed_rad_s,emf_alpha_v,emf_beta_v,phase_error_rad,locked,active,bus_voltage_v,angle_error_rad'.split(',')
report={'before':snapshot(),'captures':[]}
if report['before']['MOTOR_EN'] or report['before']['pll_experiment.active']:
    s.close();raise RuntimeError('A new encoder case requires stopped outputs and no active PLL trial')
try:
    if not 5<=args.speed<=60:raise ValueError('Speed must be 5..60 mechanical rad/s')
    write('pll_experiment.encoder_target_speed_rad_s',struct.unpack('<I',struct.pack('<f',args.speed))[0])
    write('pll_experiment.request',3);start=time.monotonic()
    for index,deadline in enumerate([.5,2.5,5.5]):
        time.sleep(max(0,deadline-(time.monotonic()-start)))
        if not snapshot()['MOTOR_EN']:break
        write('pll_experiment.capture_request',1);time.sleep(.30)
        count=read('pll_experiment.capture_count')
        if count!=512:raise RuntimeError(f'Run aborted: incomplete capture {count}/512')
        path=base.parent/(base.name+f'_{index}')
        tcl(f'dump_image {path.with_suffix(".bin").as_posix()} {addr["pll_experiment.capture"]} {512*64}')
        with path.with_suffix('.csv').open('w',newline='') as f:
            writer=csv.writer(f,lineterminator='\n');writer.writerow(headers)
            writer.writerows(struct.iter_unpack('<16f',path.with_suffix('.bin').read_bytes()))
        report['captures'].append(str(path.with_suffix('.csv')))
        report[f'window_{index}']=snapshot()
    time.sleep(max(0,7-(time.monotonic()-start)))
    report['before_stop']=snapshot()
finally:
    write('pll_experiment.request',2);time.sleep(.10)
    report['after_stop']=snapshot()
    if report['after_stop']['MOTOR_EN'] or report['after_stop']['TIM8_MOE']:
        tcl('write_memory 0x40021018 32 {2097152}');tcl('mmw 0x40010444 0 0x8000')
        report['forced_stop']=snapshot()
    base.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    s.close()
print(json.dumps(report,indent=2))
