#!/usr/bin/env python3
"""Enable OpenOCD RTT channel 0 and save its nonblocking diagnostic text."""
import socket,time,argparse,pathlib,subprocess
from check_image import check_image
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=3);p.add_argument('--output',default='build/pll_rtt.log')
p.add_argument('--elf',default='build/PLLRelease/project.elf');a=p.parse_args()
s=socket.create_connection(('localhost',6666),timeout=5)
def tcl(cmd):
    s.sendall(cmd.encode()+b'\x1a');b=b''
    while not b.endswith(b'\x1a'):
        chunk=s.recv(4096)
        if not chunk: raise RuntimeError('OpenOCD closed Tcl connection')
        b+=chunk
    return b[:-1].decode()
check_image(tcl,a.elf)
nm=subprocess.check_output(['arm-none-eabi-nm','-S',a.elf],text=True)
symbol=next(line.split() for line in nm.splitlines() if line.split()[-1]=='_SEGGER_RTT')
address=int(symbol[0],16);size=int(symbol[1],16)
if not 0x20000000<=address<address+size<=0x20020000:raise RuntimeError('RTT block outside SRAM')
identifier=bytes(int(x,0) for x in tcl(f'read_memory {address} 8 16').split())
if not identifier.startswith(b'SEGGER RTT'):raise RuntimeError('Matching firmware has not initialized RTT')
# Stop stale polling and the old channel server after an image/layout change.
print(tcl('rtt stop'));print(tcl('rtt server stop 9090'))
print(tcl(f'rtt setup {address} {size} "SEGGER RTT"'))
print(tcl('rtt start'))
print(tcl('rtt server start 9090 0'));s.close()
stream=socket.create_connection(('localhost',9090),timeout=5);stream.settimeout(.5)
end=time.monotonic()+a.seconds;data=b''
while time.monotonic()<end:
    try:
        chunk=stream.recv(4096)
        if not chunk: break
        data+=chunk
    except TimeoutError:pass
stream.close();path=pathlib.Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
if not data or b'\x00' in data or b'PLL ' not in data:
    raise RuntimeError('RTT data is not a valid PLL text log; raw bytes preserved, do not use as run evidence')
print(data.decode(errors='replace'));print(path)
