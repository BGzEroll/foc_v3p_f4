#!/usr/bin/env python3
"""Enable OpenOCD RTT channel 0 and save its nonblocking diagnostic text."""
import socket,time,argparse,pathlib
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=3);p.add_argument('--output',default='build/pll_rtt.log');a=p.parse_args()
s=socket.create_connection(('localhost',6666),timeout=5)
def tcl(cmd):
    s.sendall(cmd.encode()+b'\x1a');b=b''
    while not b.endswith(b'\x1a'):
        chunk=s.recv(4096)
        if not chunk: raise RuntimeError('OpenOCD closed Tcl connection')
        b+=chunk
    return b[:-1].decode()
print(tcl('rtt setup 0x20000000 131072 "SEGGER RTT"'))
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
print(data.decode(errors='replace'));print(path)
