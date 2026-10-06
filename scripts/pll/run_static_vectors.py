#!/usr/bin/env python3
"""Six 3 V static vectors, 0.7 s each. Full scan captured at 100 Hz.
Does not infer calibration or change current scale; outputs stop at 4.2 s.
"""
import argparse,subprocess,socket,time,pathlib,json,re
from check_image import check_image
p=argparse.ArgumentParser();p.add_argument('--elf',default='build/PLLRelease/project.elf');p.add_argument('--output',required=True);a=p.parse_args()
fields=['pll_experiment.request','pll_experiment.capture_stride','pll_experiment.capture_request',
 'pll_experiment.direct_mode','motor_instance.state_','pll_experiment.fault']
g=['gdb-multiarch','-q','-batch',a.elf]
for f in fields:g+=['-ex','p/x &'+f]
addrs=re.findall(r'= (0x[0-9a-f]+)',subprocess.check_output(g,text=True))
if len(addrs)!=len(fields):raise RuntimeError('Missing symbols')
addr=dict(zip(fields,[int(x,16) for x in addrs]));s=socket.create_connection(('localhost',6666),timeout=10)
def t(c):
 s.sendall(c.encode()+b'\x1a');b=b''
 while not b.endswith(b'\x1a'):
  chunk=s.recv(65536)
  if not chunk:raise RuntimeError('OpenOCD disconnected')
  b+=chunk
 return b[:-1].decode()
def r(f):return int(t(f'read_memory {addr[f]} {8 if f=="motor_instance.state_" else 32} 1').strip(),0)
def w(f,n):
 result=t(f'write_memory {addr[f]} 32 {{{n}}}')
 if result:raise RuntimeError(result)
check_image(t,a.elf)
if r('pll_experiment.direct_mode')!=1 or r('motor_instance.state_')!=3 or r('pll_experiment.fault') or int(t('read_memory 0x40021014 32 1').strip(),0)&32:
 raise RuntimeError('Requires direct READY and stopped output')
try:
 w('pll_experiment.capture_stride',200);w('pll_experiment.capture_request',1);w('pll_experiment.request',5)
 time.sleep(4.7)
finally:
 w('pll_experiment.request',2);time.sleep(.1)
 t('write_memory 0x40021018 32 {2097152}');t('mmw 0x40010444 0 0x8000');s.close()
subprocess.run(['python3','scripts/pll/capture_board.py','--elf',a.elf,'--output',a.output,'--read-existing','--allow-partial'],check=True)
