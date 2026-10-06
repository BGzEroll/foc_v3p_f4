#!/usr/bin/env python3
"""Explicit bounded direct sensorless start; optional encoder sampling disabled.
No reset/flash/halt. Firmware timeout <=18 s total, <=10 s PLL feedback.
"""
import argparse,csv,json,pathlib,re,socket,struct,subprocess,time
from check_image import check_image
p=argparse.ArgumentParser();p.add_argument('--elf',default='build/PLLRelease/project.elf')
p.add_argument('--output',required=True);p.add_argument('--no-encoder',action='store_true');a=p.parse_args()
base=pathlib.Path(a.output);base.parent.mkdir(parents=True,exist_ok=True)
formats={'pll_experiment.request':'I','pll_experiment.direct_mode':'I','pll_experiment.fault':'I',
 'pll_experiment.active':'I','pll_experiment.active_samples':'I','pll_experiment.samples':'I',
 'pll_experiment.capture_request':'I','pll_experiment.capture_count':'I','pll_experiment.capture':'I',
 'pll_experiment.max_loop_cycles':'I','pll_experiment.max_period_us':'I','pll_experiment.missed_periods':'I',
 'pll_encoder_diagnostics_enabled':'I','rotor.sequence':'I','rotor.error_count':'I',
 'Sguan.status':'B','motor_instance.state_':'B','Sguan.current.Real_Id':'f','Sguan.current.Real_Iq':'f',
 'Sguan.encoder.Real_Espeed':'f'}
for n in ['state','reason','ticks','stage_ticks','qualified_ticks','closed_ticks']:
 formats['pll_experiment.startup.'+n]='I'
for n in ['open_angle','open_speed','control_angle','control_speed','angle_difference','target_id','target_iq','max_phase_a']:
 formats['pll_experiment.startup.'+n]='f'
g=['gdb-multiarch','-q','-batch',a.elf]
for f in formats:g+=['-ex','p/x &'+f]
addresses=re.findall(r'= (0x[0-9a-f]+)',subprocess.check_output(g,text=True))
if len(addresses)!=len(formats):raise RuntimeError('Missing diagnostic symbols')
addr=dict(zip(formats,[int(x,16) for x in addresses]));s=socket.create_connection(('localhost',6666),timeout=10)
def tcl(c):
 s.sendall(c.encode()+b'\x1a');b=b''
 while not b.endswith(b'\x1a'):
  q=s.recv(65536)
  if not q:raise RuntimeError('OpenOCD disconnected')
  b+=q
 return b[:-1].decode()
def read(f):
 fmt=formats[f];width=8 if fmt=='B' else 32
 n=int(tcl(f'read_memory {addr[f]} {width} 1').strip(),0)
 return struct.unpack('<'+fmt,n.to_bytes(width//8,'little'))[0]
def write(f,n):
 r=tcl(f'write_memory {addr[f]} 32 {{{n}}}')
 if r:raise RuntimeError(r)
def hw():
 return {'MOTOR_EN':bool(int(tcl('read_memory 0x40021014 32 1').strip(),0)&32),
 'TIM8_MOE':bool(int(tcl('read_memory 0x40010444 32 1').strip(),0)&32768)}
def snapshot():
 d={f:read(f) for f in formats if f!='pll_experiment.capture'};d.update(hw());return d
check_image(tcl,a.elf);report={'before':snapshot(),'no_encoder':a.no_encoder,'captures':[],'timeline':[]}
if read('pll_experiment.direct_mode')!=1 or read('motor_instance.state_')!=3 or read('pll_experiment.fault') or hw()['MOTOR_EN']:
 s.close();raise RuntimeError('Requires direct-mode READY with output stopped and no fault')
headers='time_s,v_alpha_v,v_beta_v,i_alpha_a,i_beta_a,encoder_angle_rad,encoder_speed_rad_s,pll_angle_rad,pll_speed_rad_s,emf_alpha_v,emf_beta_v,phase_error_rad,locked,active,bus_voltage_v,angle_error_rad'.split(',')
try:
 if a.no_encoder:write('pll_encoder_diagnostics_enabled',0);time.sleep(.05)
 report['at_start']=snapshot();write('pll_experiment.request',4);start=time.monotonic()
 for index,deadline in enumerate([.2,1.2,2.4,3.6,4.6,6.5,9,12]):
  time.sleep(max(0,deadline-(time.monotonic()-start)))
  row=snapshot();row['host_elapsed_s']=time.monotonic()-start;report['timeline'].append(row)
  if not row['MOTOR_EN']:break
  write('pll_experiment.capture_request',1);time.sleep(.3)
  count=read('pll_experiment.capture_count')
  if read('pll_experiment.capture_request') or count==0:break
  if count!=512:
   first=read('pll_experiment.samples');time.sleep(.03)
   if first!=read('pll_experiment.samples'):raise RuntimeError('Incomplete buffer still changing')
  path=base.parent/(base.name+f'_{index}')
  result=tcl(f'dump_image {path.with_suffix(".bin").as_posix()} {addr["pll_experiment.capture"]} {count*64}')
  if 'error' in result.lower():raise RuntimeError(result)
  with path.with_suffix('.csv').open('w',newline='') as f:
   writer=csv.writer(f,lineterminator='\n');writer.writerow(headers)
   writer.writerows(struct.iter_unpack('<16f',path.with_suffix('.bin').read_bytes()))
  report['captures'].append({'path':str(path.with_suffix('.csv')),'rows':count,'complete':count==512})
  if count!=512:break
 # Observe the firmware's own 10 s closed-loop / 18 s startup timeout.
 while time.monotonic()-start<18.5 and hw()['MOTOR_EN']:
  time.sleep(.25)
 report['before_stop']=snapshot()
finally:
 try:
  write('pll_experiment.request',2);time.sleep(.1)
  report['after_stop']=snapshot()
 finally:
  tcl('write_memory 0x40021018 32 {2097152}');tcl('mmw 0x40010444 0 0x8000')
  report['hardware_final']=hw();s.close()
  base.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
assert not report['hardware_final']['MOTOR_EN'] and not report['hardware_final']['TIM8_MOE']
