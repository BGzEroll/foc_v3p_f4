#!/usr/bin/env python3
"""Read experiment status without halting the CPU (matching ELF needed)."""
import socket,subprocess,re,struct,json,argparse
p=argparse.ArgumentParser();p.add_argument('--elf',default='build/PLLRelease/project.elf');args=p.parse_args()
sock=socket.create_connection(('localhost',6666),timeout=10)
def tcl(cmd):
    sock.sendall(cmd.encode()+b'\x1a');buf=b''
    while not buf.endswith(b'\x1a'):
        chunk=sock.recv(4096)
        if not chunk: raise RuntimeError('OpenOCD closed Tcl connection')
        buf+=chunk
    return buf[:-1].decode()
fields=[('motor_instance.state_',8,'B'),('motor_instance.last_result_',8,'B'),
 ('motor_instance.output_enabled_',8,'B'),('Sguan.status',8,'B'),
 ('rotor.sequence',32,'I'),('rotor.error_count',32,'I'),('rotor.previous_timestamp_us',32,'I'),
 ('uwTick',32,'I'),('pll_experiment.samples',32,'I'),('pll_experiment.max_loop_cycles',32,'I'),
 ('pll_experiment.last_period_us',32,'I'),('pll_experiment.active',32,'I'),('pll_experiment.fault',32,'I'),
 ('pll_experiment.encoder_fault_age_us',32,'I'),('pll_experiment.encoder_fault_sample_us',32,'I'),('pll_experiment.encoder_fault_now_us',32,'I'),
 ('pll_experiment.missed_periods',32,'I'),('pll_experiment.max_period_us',32,'I'),
 ('Sguan.encoder.Real_Espeed',32,'f'),('Sguan.current.Real_Id',32,'f'),('Sguan.current.Real_Iq',32,'f'),
 ('Sguan.foc.Real_VBUS',32,'f')]
data={}
for expr,width,fmt in fields:
    out=subprocess.check_output(['gdb-multiarch','-q','-batch',args.elf,'-ex',f'p/x &{expr}'],text=True)
    addr=int(re.search(r'= (0x[0-9a-f]+)',out).group(1),16)
    value=int(tcl(f'read_memory {addr} {width} 1').strip(),0)
    data[expr]=struct.unpack('<'+fmt,value.to_bytes(width//8,'little'))[0]
data['TIM5_CNT']=int(tcl('read_memory 0x40000c24 32 1').strip(),0)
data['TIM8_BDTR']=int(tcl('read_memory 0x40010444 32 1').strip(),0)
data['MOTOR_EN']=int(tcl('read_memory 0x40021014 32 1').strip(),0)&0x20 != 0
print(json.dumps(data,indent=2));sock.close()
