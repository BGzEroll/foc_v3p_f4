#!/usr/bin/env python3
"""Read experiment status without halting the CPU (matching ELF needed)."""
import socket,subprocess,re,struct,json,argparse
from check_image import check_image
p=argparse.ArgumentParser();p.add_argument('--elf',default='build/PLLRelease/project.elf');args=p.parse_args()
sock=socket.create_connection(('localhost',6666),timeout=10)
def tcl(cmd):
    sock.sendall(cmd.encode()+b'\x1a');buf=b''
    while not buf.endswith(b'\x1a'):
        chunk=sock.recv(4096)
        if not chunk: raise RuntimeError('OpenOCD closed Tcl connection')
        buf+=chunk
    return buf[:-1].decode()
check_image(tcl,args.elf)
fields=[('motor_instance.state_',8,'B'),('motor_instance.last_result_',8,'B'),
 ('motor_instance.output_enabled_',8,'B'),('Sguan.status',8,'B'),
 ('rotor.sequence',32,'I'),('rotor.error_count',32,'I'),('rotor.previous_timestamp_us',32,'I'),
 ('rotor.consecutive_errors',32,'I'),('rotor.last_i2c_result',32,'I'),('rotor.last_success_us',32,'I'),
 ('rotor.config_before',32,'I'),('rotor.config_after',32,'I'),('rotor.magnet_status',32,'I'),
 ('rotor.magnet_agc',32,'I'),('rotor.magnet_magnitude',32,'I'),
 ('rotor.last_read_duration_us',32,'I'),('rotor.max_read_duration_us',32,'I'),
 ('i2c_bus_debug[0].recovery_attempts',32,'I'),('i2c_bus_debug[0].recovery_failures',32,'I'),
 ('i2c_bus_debug[0].transfers_ok',32,'I'),('i2c_bus_debug[0].transfers_failed',32,'I'),
 ('i2c_bus_debug[0].last_hal_error',32,'I'),('i2c_bus_debug[0].last_lines_before',32,'I'),('i2c_bus_debug[0].last_lines_after',32,'I'),
 ('uwTick',32,'I'),('pll_experiment.samples',32,'I'),('pll_experiment.max_loop_cycles',32,'I'),
 ('pll_experiment.last_period_us',32,'I'),('pll_experiment.active',32,'I'),('pll_experiment.fault',32,'I'),
 ('pll_experiment.encoder_fault_age_us',32,'I'),('pll_experiment.encoder_fault_sample_us',32,'I'),('pll_experiment.encoder_fault_now_us',32,'I'),
 ('pll_experiment.missed_periods',32,'I'),('pll_experiment.max_period_us',32,'I'),
 ('pll_experiment.startup_encoder_delta_rad',32,'f'),('pll_experiment.startup_max_phase_a',32,'f'),
 ('pll_experiment.startup_reverse_delta_rad',32,'f'),('pll_experiment.startup_zero_spread_rad',32,'f'),
 ('pll_experiment.encoder_target_speed_rad_s',32,'f'),('pll_experiment.encoder_current_limit_a',32,'f'),('pll_experiment.encoder_iq_command_a',32,'f'),
 ('pll_experiment.switch_good',32,'I'),('pll_experiment.max_switch_good',32,'I'),('pll_experiment.active_samples',32,'I'),
 ('Sguan.motor.Encoder_Dir',8,'b'),('Sguan.encoder.Pos_offset',32,'f'),
 ('Sguan.encoder.Real_Espeed',32,'f'),('Sguan.current.Real_Id',32,'f'),('Sguan.current.Real_Iq',32,'f'),
 ('Sguan.foc.Real_VBUS',32,'f')]
data={}
fields += [('pll_experiment.direct_mode',32,'I'),('pll_encoder_diagnostics_enabled',32,'I'),
 ('pll_experiment.startup.state',32,'I'),('pll_experiment.startup.reason',32,'I'),
 ('pll_experiment.startup.closed_ticks',32,'I'),('pll_experiment.startup.max_phase_a',32,'f'),
 ('pll_experiment.estimator.config.resistance_ohm',32,'f'),
 ('current_sense.config.phase_mapping',8,'B'),('current_sense.config.direction_a',8,'b'),('current_sense.config.direction_b',8,'b')]
gdb=['gdb-multiarch','-q','-batch',args.elf]
for expr,_,_ in fields:gdb += ['-ex',f'p/x &{expr}']
addresses=re.findall(r'= (0x[0-9a-f]+)',subprocess.check_output(gdb,text=True))
if len(addresses)!=len(fields):raise RuntimeError('Cannot resolve diagnostic symbols')
for (expr,width,fmt),address in zip(fields,addresses):
    addr=int(address,16)
    value=int(tcl(f'read_memory {addr} {width} 1').strip(),0)
    data[expr]=struct.unpack('<'+fmt,value.to_bytes(width//8,'little'))[0]
data['TIM5_CNT']=int(tcl('read_memory 0x40000c24 32 1').strip(),0)
data['TIM8_BDTR']=int(tcl('read_memory 0x40010444 32 1').strip(),0)
data['MOTOR_EN']=int(tcl('read_memory 0x40021014 32 1').strip(),0)&0x20 != 0
print(json.dumps(data,indent=2));sock.close()
