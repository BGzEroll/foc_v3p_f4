#!/usr/bin/env python3
"""Read a coherent frozen capture over OpenOCD Tcl without halting the motor.
Run from repo root. Requires a running OpenOCD with the matching flashed ELF.
Usage: python3 scripts/pll/capture_board.py --output build/pll_capture
Use --request-switch only after inspecting shadow errors; --stop closes output.
"""
import argparse, pathlib, socket, subprocess, struct, time, json
from check_image import check_image
p=argparse.ArgumentParser()
p.add_argument('--output',default='build/pll_capture')
p.add_argument('--elf',default='build/PLLRelease/project.elf')
action=p.add_mutually_exclusive_group()
action.add_argument('--request-switch',action='store_true')
action.add_argument('--stop',action='store_true')
action.add_argument('--start-encoder',action='store_true',help='Start the configured encoder baseline after boot initialization')
p.add_argument('--mapping',type=int,choices=[0,1],help='Shadow observer wiring diagnosis: 0 normal, 1 swap A/C')
p.add_argument('--read-existing',action='store_true',help='Download the frozen buffer without a new capture request')
p.add_argument('--allow-partial',action='store_true',help='Save an aborted frozen buffer as explicitly incomplete diagnostic data')
args=p.parse_args()
nm=subprocess.check_output(['arm-none-eabi-nm',args.elf],text=True)
addr=int(next(line.split()[0] for line in nm.splitlines() if line.split()[-1]=='pll_experiment'),16)
sock=socket.create_connection(('localhost',6666),timeout=10)
def tcl(command):
    sock.sendall(command.encode()+b'\x1a')
    buf=b''
    while not buf.endswith(b'\x1a'):
        chunk=sock.recv(65536)
        if not chunk: raise RuntimeError('OpenOCD closed Tcl connection')
        buf+=chunk
    result=buf[:-1].decode()
    return result
def field_offset(name):
    # GDB is used OFFLINE solely for structure layout, no target connection.
    text=subprocess.check_output(['gdb-multiarch','-q','-batch',args.elf,
        '-ex',f'p/d (unsigned long)&((pll_experiment_state*)0)->{name}'],text=True)
    return int(text.split('=')[-1].strip())
def write_field(name,value):
    result=tcl(f'write_memory {addr+field_offset(name)} 32 {{{value}}}')
    if result: raise RuntimeError(result)
# Check the whole programmed image BEFORE any RAM writes. A different build can
# put globals at different addresses even when the C structures look identical.
check_image(tcl,args.elf)
if args.mapping is not None:
    active=int(tcl(f'read_memory {addr+field_offset("active")} 32 1').strip(),0)
    if active: raise RuntimeError('Cannot change observer mapping during an active sensorless trial')
    write_field('current_mapping',args.mapping)
    time.sleep(1)
if args.start_encoder:
    write_field('request',3)
    time.sleep(1)
if args.stop:
    write_field('request',2)
    time.sleep(0.1)
    print('Stop requested. Confirm MOTOR_EN=0 and TIM8.MOE=0 using board_status.py.')
    sock.close()
    raise SystemExit(0)
if args.request_switch: write_field('request',1)
if not args.read_existing:
    write_field('capture_request',1)
    time.sleep(0.6)
count=int(tcl(f'read_memory {addr+field_offset("capture_count")} 32 1').strip(),0)
if count != 512:
    if not(args.read_existing and args.allow_partial and 0<count<512):
        raise RuntimeError(f'Capture is incomplete ({count}/512), no accepted run samples')
    seq_addr=addr+field_offset('samples')
    before=tcl(f'read_memory {seq_addr} 32 1');time.sleep(.03)
    if before!=tcl(f'read_memory {seq_addr} 32 1'):raise RuntimeError('Partial buffer is still being written')
    print(f'PARTIAL diagnostic capture: {count}/512 rows')
path=pathlib.Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
capture_addr=addr+field_offset('capture')
result=tcl(f'dump_image {path.with_suffix(".bin").as_posix()} {capture_addr} {count*16*4}')
print(result)
raw=path.with_suffix('.bin').read_bytes()
rows=struct.iter_unpack('<16f',raw)
headers='time_s,v_alpha_v,v_beta_v,i_alpha_a,i_beta_a,encoder_angle_rad,encoder_speed_rad_s,pll_angle_rad,pll_speed_rad_s,emf_alpha_v,emf_beta_v,phase_error_rad,locked,active,bus_voltage_v,angle_error_rad'
with path.with_suffix('.csv').open('w') as f:
    f.write(headers+'\n')
    for row in rows: f.write(','.join(str(v) for v in row)+'\n')
metadata={}
for name in ['samples','last_period_us','max_step_cycles','max_loop_cycles','capture_count','active','fault','request','switch_good','current_mapping','missed_periods','max_period_us']:
    metadata[name]=int(tcl(f'read_memory {addr+field_offset(name)} 32 1').strip(),0)
metadata['step_max_us']=metadata['max_step_cycles']/168
metadata['capture_complete']=count==512
metadata['loop_max_us']=metadata['max_loop_cycles']/168
path.with_suffix('.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps(metadata,indent=2));print(path.with_suffix('.csv'))
sock.close()
