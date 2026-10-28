"""Offline tests of the actual fallback ELF, never a port/probe/device.

M7 tests execute actual control, PWM and IWDG register instructions with a
minimal timer/GPIO model. Unicorn's M7 cannot execute DP VFP, so math/tracking
tests use ARM_MAX with only the hardware boundary substituted. Both use the
same production ELF; neither proves physical timing, loads or shaft position.
"""
from pathlib import Path
import os
import struct
import sys
import unittest

if os.environ.get('OASIS_EMULATION_DEPS'):
    sys.path.insert(0, os.environ['OASIS_EMULATION_DEPS'])
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (UC_CPU_ARM_CORTEX_M7, UC_CPU_ARM_MAX, UC_ARM_REG_SP,
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2,
    UC_ARM_REG_R3, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_PRIMASK)
from unicorn.arm_const import UC_ARM_REG_D0, UC_ARM_REG_D1

ROOT=Path(__file__).resolve().parents[1]
ELF=ROOT/'build_fallback/fallback_control_h723.elf'
sys.path.insert(0,str(ROOT.parent))
from experiment_a.cad_limits import (
    CAD_CAP_POINTS, CAD_INFO_LINE, CAD_PITCH_MAX_MDEG, CAD_YAW_MAX_MDEG,
    pose_within_cad,
)
PE,TIM1,RCC,DBG,IWDG=0x58021000,0x40010000,0x58024400,0x5C001000,0x58004800
PINS=sum(1<<pin for pin in (13,14,11,9))
MODE_MASK=sum(3<<(2*pin) for pin in (13,14,11,9))
OUTPUT_MODE=sum(1<<(2*pin) for pin in (13,14,11,9))
CCR=(0x3C,0x40,0x38,0x34)
NEUTRAL=(1800,1800,1200,1200)
DEBUG_SIZE=148
TUNE_FIELDS=('yaw_max_mdeg','pitch_max_mdeg','yaw_rate_mdeg_s','pitch_rate_mdeg_s',
             'full_scale_ay_mmps2','pitch_scale_permille')
TUNE_DEFAULTS=(15000,6000,2000,2000,2942,1000)


class Machine:
    def __init__(self, *, math=False, initialize=True, now=0):
        self.math=math
        self.uc=Uc(UC_ARCH_ARM,UC_MODE_THUMB | (0 if math else UC_MODE_MCLASS))
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_MAX if math else UC_CPU_ARM_CORTEX_M7)
        for address,size in ((0x08000000,0x100000),(0x20000000,0x20000),
            (0x24000000,0x80000),(0x30000000,0x80000),(0x40000000,0x30000),
            (0x58000000,0x100000),(DBG,0x1000),(0xE0000000,0x100000)):
            self.uc.mem_map(address,size)
        data=ELF.read_bytes()
        header=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
        assert data[:6]==b'\x7fELF\x01\x01'
        sections=[struct.unpack_from('<10I',data,header[6]+i*header[11]) for i in range(header[12])]
        for section in sections:
            if section[2]&2 and section[1]!=8 and section[5]:
                self.uc.mem_write(section[3],data[section[4]:section[4]+section[5]])
        self.symbols={}
        for section in sections:
            if section[1]!=2: continue
            string_section=sections[section[6]]
            names=data[string_section[4]:string_section[4]+string_section[5]]
            for offset in range(section[4],section[4]+section[5],section[9]):
                name,value,_,_,_,_=struct.unpack_from('<IIIBBH',data,offset)
                self.symbols[names[name:].split(b'\0',1)[0].decode('ascii')]=value
        self.uc.reg_write(UC_ARM_REG_C1_C0_2,0x00F00000)
        if math: self.uc.reg_write(UC_ARM_REG_FPEXC,1<<30)
        self.now=now & 0xffffffff
        self.tx=[]; self.writes=[]; self.regs={}; self.active=(0,0,0,0); self.frames=[]
        self.force_rollover=False; self.shadow=(0,0,0,0); self.enabled=False
        self.frame_ready=False; self.outputs=[]; self.watchdog_refreshes=0
        self.hardware_valid=True; self.transmit_hook=None
        self.hooks={}
        for name in ('HAL_GetTick','HAL_RCC_GetPCLK2Freq','FSR_Transmit','FSR_TryTelemetry',
                     'HAL_UART_Transmit','HAL_UART_Transmit_IT','HAL_UART_AbortReceive',
                     'HAL_UART_Receive_IT','HAL_UART_AbortTransmit'):
            if name in self.symbols: self.hooks[self.symbols[name]&~1]=name
        if math:
            for name in ('Fallback_MotorOff','Fallback_PwmStart','Fallback_PwmValid',
                         'Fallback_PwmFrameReady','Fallback_PwmStage',
                         'Fallback_WatchdogStart','Fallback_WatchdogRefresh'):
                if name in self.symbols: self.hooks[self.symbols[name]&~1]=name
        self.uc.hook_add(UC_HOOK_CODE,self.code)
        self.uc.hook_add(UC_HOOK_MEM_WRITE,self.write)
        self.word(self.symbols['SystemCoreClock'],64000000)
        self.word(RCC+0x74,2)
        self.word(DBG+0x4C,1); self.word(DBG+0x54,1<<18)
        if initialize: self.call('Control_Init')

    def word(self,address,value=None):
        if value is None: return struct.unpack('<I',self.uc.mem_read(address,4))[0]
        self.uc.mem_write(address,struct.pack('<I',value&0xffffffff))
        if TIM1<=address<TIM1+0x80: self.regs[address-TIM1]=value&0xffffffff

    def cstring(self,address):
        result=bytearray()
        while True:
            byte=self.uc.mem_read(address+len(result),1)[0]
            if not byte: return result.decode('ascii')
            result.append(byte)

    def code(self,uc,address,size,user):
        name=self.hooks.get(address)
        if not name: return
        args=[uc.reg_read(reg) for reg in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)]
        result=0
        if name=='HAL_GetTick': result=self.now
        elif name=='HAL_RCC_GetPCLK2Freq': result=64000000
        elif name in ('FSR_Transmit','FSR_TryTelemetry','HAL_UART_Transmit','HAL_UART_Transmit_IT'):
            self.tx.append(self.cstring(args[1] if name.startswith('HAL_') else args[0]).rstrip('\r\n'))
            if name=='FSR_TryTelemetry': result=1
            if name=='HAL_UART_Transmit_IT':
                uc.mem_write(self.symbols['stats_tx_busy'],b'\0')
            if self.transmit_hook: self.transmit_hook(self)
        elif name=='Fallback_MotorOff': self.enabled=False; self.shadow=(0,0,0,0)
        elif name=='Fallback_PwmStart':
            self.shadow=struct.unpack('<4H',uc.mem_read(args[0],8)); self.enabled=True; result=1
        elif name=='Fallback_PwmValid':
            p=struct.unpack('<4H',uc.mem_read(args[0],8))
            result=int(self.hardware_valid and p==self.shadow and bool(args[1])==self.enabled)
        elif name=='Fallback_PwmFrameReady': result=int(self.frame_ready)
        elif name=='Fallback_PwmStage':
            if self.frame_ready:
                self.shadow=struct.unpack('<4H',uc.mem_read(args[0],8))
                self.outputs.append((self.now,self.shadow)); self.frame_ready=False; result=1
        elif name=='Fallback_WatchdogStart': result=1
        elif name=='Fallback_WatchdogRefresh': self.watchdog_refreshes+=1
        uc.reg_write(UC_ARM_REG_R0,result)
        uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))

    def rollover(self):
        if not self.regs.get(0,0)&2:
            self.active=tuple(self.regs.get(r,0) for r in CCR)
            self.word(TIM1+0x10,self.regs.get(0x10,0)|1)
        self.word(TIM1+0x24,0)
        self.frames.append(self.active)

    def write(self,uc,access,address,size,value,user):
        self.writes.append((address,value))
        if address==PE+0x18:
            odr=(self.word(PE+0x14)|(value&0xffff))&~(value>>16)
            self.word(PE+0x14,odr); self.word(PE+0x10,odr)
        if TIM1<=address<TIM1+0x80:
            offset=address-TIM1; self.regs[offset]=value
            if offset==0x14 and value&1: self.rollover()
            if self.force_rollover and offset in CCR and self.regs.get(0,0)&1: self.rollover()
        if address==IWDG and value==0xAAAA: self.watchdog_refreshes+=1

    def call(self,name,*args):
        self.uc.reg_write(UC_ARM_REG_SP,0x2407F000)
        self.uc.reg_write(UC_ARM_REG_LR,0x080FF001)
        for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):
            self.uc.reg_write(reg,value&0xffffffff)
        self.uc.emu_start(self.symbols[name]|1,0x080FF000,count=5000000)
        if self.uc.reg_read(UC_ARM_REG_PC)!=0x080FF000: raise AssertionError(name+' exceeded instruction bound')
        return self.uc.reg_read(UC_ARM_REG_R0)

    def command(self,text):
        self.uc.mem_write(0x24060000,text.encode('ascii')+b'\0')
        start=len(self.tx); self.call('Control_Command',0x24060000)
        return self.tx[start:]

    def state(self):
        line=self.command('STATE')[-1].split(',')
        assert len(line)==18,line
        return dict(tick=int(line[1]),armed=int(line[2]),moving=int(line[3]),
                    pose=tuple(map(int,line[4:8])),target=tuple(map(int,line[8:12])),
                    pulses=tuple(map(int,line[12:16])),id=int(line[16]),fault=line[17])

    def debug_snapshot(self):
        """Read the public observation copy, not the private control globals."""
        data=bytes(self.uc.mem_read(self.symbols['g_fallback_debug'],DEBUG_SIZE))
        words=struct.unpack('<20I',data[:80])
        signed=struct.unpack('<8i',data[28:60])
        return dict(sequence=words[0],schema_version=words[1],tick=words[2],
                    armed=words[3],moving=words[4],session=words[5],id=words[6],
                    pose=signed[:4],target=signed[4:],pulses=words[15:19],
                    lease_elapsed_ms=words[19],fault=data[80:112].split(b'\0',1)[0].decode('ascii'),
                    tuning_revision=struct.unpack_from('<I',data,112)[0],
                    tuning_handled_seq=struct.unpack_from('<I',data,116)[0],
                    tuning_result_code=struct.unpack_from('<I',data,120)[0],
                    tuning_active=struct.unpack_from('<6i',data,124))

    def tick(self,elapsed=20,frame=True):
        self.now=(self.now+elapsed)&0xffffffff
        if self.math: self.frame_ready=frame
        elif frame: self.rollover(); self.word(TIM1+0x24,5000)
        self.call('Control_Poll')

    def arm(self,sid=1,revision=None):
        if revision is None: revision=self.debug_snapshot()['tuning_revision']
        assert self.command(f'FB_ARM,{sid},{revision}')==[f'#ACK,FB_ARM,{sid},{revision}']
        return self

    def tune(self,seq=1,**changes):
        values=dict(zip(TUNE_FIELDS,TUNE_DEFAULTS)); values.update(changes)
        self.uc.mem_write(self.symbols['g_experiment_a_tune'],
                          struct.pack('<6iI',*(values[name] for name in TUNE_FIELDS),seq))
        self.call('Control_Poll')
        return self

    def inject_rx(self,payload):
        if len(payload)>127: raise ValueError('fixture ring capacity')
        self.uc.mem_write(self.symbols['rx_ring'],payload)
        self.uc.mem_write(self.symbols['rx_head'],struct.pack('<H',len(payload)))
        self.uc.mem_write(self.symbols['rx_tail'],b'\0\0')


class ControlTests(unittest.TestCase):
    def test_debug_snapshot_is_observation_only_and_matches_command_state(self):
        m=Machine(math=True)
        before=m.debug_snapshot()
        self.assertEqual(before['schema_version'],1)
        self.assertEqual(before['sequence']%2,0)
        self.assertEqual(before['pulses'],(0,)*4)
        self.assertEqual(before['fault'],'BOOT')
        # Arbitrary debugger writes must neither arm nor affect private state.
        m.uc.mem_write(m.symbols['g_fallback_debug'],b'\xff'*DEBUG_SIZE)
        m.tick()
        self.assertEqual(m.state()['armed'],0)
        self.assertEqual(m.debug_snapshot()['pulses'],(0,)*4)
        m.arm(5); m.command('FB_SET,5,1,5000,0,0,0'); m.tick()
        state=m.state(); debug=m.debug_snapshot()
        for key in ('tick','armed','moving','id','pose','target','pulses','fault'):
            self.assertEqual(debug[key],state[key],key)
        self.assertEqual(debug['session'],5)
        self.assertEqual(debug['sequence']%2,0)
        self.assertGreater(debug['sequence'],before['sequence'])
        self.assertEqual(debug['lease_elapsed_ms'],20)
        m.now+=500; m.call('Fallback_SafetyPoll')
        self.assertEqual(m.debug_snapshot()['fault'],'LEASE')
        self.assertEqual(m.debug_snapshot()['pulses'],(0,)*4)

    def test_boot_low_generic_commands_cannot_arm(self):
        m=Machine()
        for command in ('ARM','SET,1,0,0,0,0','HOLD','FB_ARM,0','FB_ARM,4294967296'):
            self.assertTrue(m.command(command)[0].startswith('#ERR,'))
            self.assertEqual(m.state()['pulses'],(0,)*4)
            self.assertEqual(m.word(PE)&MODE_MASK,OUTPUT_MODE)
        self.assertIn('OASIS_FALLBACK,v2',m.command('CONTROL_HELLO')[0])
        self.assertIn('servo_range_deg=270',m.command('CONTROL_HELLO')[0])

    def test_session_replay_fault_and_new_explicit_arm(self):
        m=Machine().arm(10)
        self.assertEqual(m.state()['pulses'],NEUTRAL)
        m.command('FB_OFF')
        self.assertEqual(m.command('FB_ARM,10,0'),['#ERR,0,SESSION_REPLAY'])
        self.assertEqual(m.command('FB_ARM,9,0'),['#ERR,0,SESSION_REPLAY'])
        m.arm(11)
        self.assertIn('session_floor=11',m.command('CONTROL_HELLO')[0])
        self.assertEqual(m.command('FB_PING,10,1'),['#ERR,0,SESSION'])
        self.assertEqual(m.state()['armed'],0)
        self.assertEqual(m.command('FB_ARM,11,0'),['#ERR,0,SESSION_REPLAY'])
        m.arm(12)

    def test_pressure_heartbeat_and_expired_ping_do_not_revive(self):
        m=Machine(now=0xFFFFFF00).arm()
        m.now=(m.now+499)&0xffffffff; m.call('Control_Heartbeat'); m.call('Control_Poll')
        self.assertEqual(m.state()['armed'],1)
        m.now=(m.now+1)&0xffffffff
        self.assertEqual(m.command('FB_PING,1,1'),['#ERR,0,DISARMED'])
        self.assertEqual(m.state()['fault'],'LEASE')
        self.assertEqual(m.state()['pulses'],(0,)*4)

    def test_dedicated_ping_counter_monotonic_and_off(self):
        m=Machine().arm()
        m.now=400
        self.assertEqual(m.command('FB_PING,1,1'),['#ACK,FB_PING,1,1'])
        m.now=800; self.assertEqual(m.state()['armed'],1)
        self.assertEqual(m.command('FB_PING,1,1'),['#ERR,0,PING_REPLAY'])
        self.assertEqual(m.state()['armed'],0)
        m.command('FB_OFF'); self.assertEqual(m.word(TIM1+0x20),0)

    def test_tracking_continues_under_5ms_set_stream_and_two_us_bound(self):
        m=Machine(math=True).arm()
        old=NEUTRAL
        for i in range(1,81):
            m.now=i*5
            ack=m.command(f'FB_SET,1,{i},5000,0,0,0')
            self.assertEqual(ack,[f'#ACK,FB_SET,1,{i},5000,0,0,0'])
            m.frame_ready=(i%4==0); m.call('Control_Poll')
            new=m.state()['pulses']
            self.assertTrue(all(abs(a-b)<=2 for a,b in zip(new,old)))
            old=new
        self.assertGreater(len(m.outputs),10)
        self.assertGreater(m.state()['pose'][0],0)
        self.assertEqual(m.state()['pulses'][0],1800)
        self.assertEqual(m.state()['pulses'][2],1200)

    def test_command_ids_bounds_and_malformed_fail_closed(self):
        bad=('FB_SET,1,0,0,0,0,0','FB_SET,1,1,-1,0,0,0','FB_SET,1,1,27001,0,0,0',
             'FB_SET,1,1,0,8001,0,0','FB_SET,1,1,0,0,0,0,0','FB_SET,1,1,NaN,0,0,0',
             'FB_SET,1,4294967296,0,0,0,0','FB_SET,2,1,0,0,0,0')
        for text in bad:
            with self.subTest(text=text):
                m=Machine(math=True).arm(); self.assertTrue(m.command(text)[0].startswith('#ERR,'))
                self.assertEqual(m.state()['armed'],0)
        m=Machine(math=True).arm(); m.command('FB_SET,1,2,1000,0,0,0')
        self.assertEqual(m.command('FB_SET,1,2,0,0,0,0'),['#ERR,2,SET_REPLAY'])
        self.assertEqual(m.state()['armed'],0)

    def test_gap_has_one_step_and_never_catches_up(self):
        m=Machine(math=True).arm(); m.command('FB_SET,1,1,10000,0,0,0')
        m.tick(400)
        self.assertEqual(len(m.outputs),1)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(m.shadow,NEUTRAL)),2)
        for _ in range(20): m.tick(0)
        self.assertEqual(len(m.outputs),1)
        m.tick(100)
        self.assertEqual(m.state()['armed'],0)

    def test_readback_corruption_is_not_hidden_by_due_update(self):
        m=Machine(math=True).arm(); m.command('FB_SET,1,1,1000,0,0,0')
        m.shadow=(1799,1200,1200,1800); m.tick()
        self.assertEqual(m.state()['fault'],'HARDWARE')
        self.assertEqual(m.outputs,[])

    def test_full_two_side_targets_reach_and_retarget_without_oversized_steps(self):
        m=Machine(math=True).tune(yaw_max_mdeg=CAD_YAW_MAX_MDEG,
            yaw_rate_mdeg_s=20000,pitch_rate_mdeg_s=20000).arm(); old=NEUTRAL; counter=0
        # The new hard CAD envelope intentionally rejects the old 20deg/8deg
        # target. Exercise its two signed corners and tapered yaw endpoint.
        targets=((15000,6000,15000,-6000),(20000,2000,20000,-2000),
                 (CAD_YAW_MAX_MDEG,0,CAD_YAW_MAX_MDEG,0),
                 (15000,6000,15000,-6000),(0,0,0,0))
        for command_id,target in enumerate(targets,1):
            self.assertEqual(m.command('FB_SET,1,'+str(command_id)+','+','.join(map(str,target)))[0],
                             '#ACK,FB_SET,1,'+str(command_id)+','+','.join(map(str,target)))
            for step in range(800):
                if step%10==0:
                    counter+=1; m.command(f'FB_PING,1,{counter}')
                m.tick(); current=m.state()
                self.assertEqual(current['armed'],1,current)
                self.assertTrue(pose_within_cad(current['pose']),current)
                self.assertTrue(all(abs(a-b)<=2 for a,b in zip(current['pulses'],old)))
                old=current['pulses']
                if not current['moving']: break
            else: self.fail(f'target did not converge: {current}')
            self.assertEqual(current['pose'],target)
        self.assertEqual(m.state()['pulses'],NEUTRAL)

    def test_fsr_invalid_command_and_adc_fault_remove_output(self):
        m=Machine().arm(); m.inject_rx(b'invalid\n'); m.call('FSR_Poll')
        self.assertEqual(m.state()['armed'],0)
        self.assertEqual(m.state()['fault'],'FSR_FAULT')
        m.arm(2)
        m.uc.mem_write(0x24060000,b'FSR_FAULT\0'); m.call('Control_Fault',0x24060000)
        self.assertEqual(m.state()['pulses'],(0,)*4)

    def test_lease_expiry_inside_set_math_never_restores_authority(self):
        m=Machine(math=True).arm(); m.now=499
        # A computation can consume time after the command's first lease check.
        # The real tracked math is fast in emulation; hook its boundary to expose
        # whether acceptance rechecks expiry before changing last_alive.
        address=m.symbols['Joint_PosePulses']&~1
        handle=m.uc.hook_add(UC_HOOK_CODE,lambda *_: setattr(m,'now',501),begin=address,end=address)
        replies=m.command('FB_SET,1,1,1000,0,0,0')
        self.assertEqual(m.state()['armed'],0)
        self.assertEqual(replies,['#ERR,1,DISARMED'])

    def test_corruption_during_candidate_math_cannot_be_overwritten(self):
        m=Machine(math=True).arm(); m.command('FB_SET,1,1,1000,0,0,0')
        address=m.symbols['Joint_PoseStep']&~1
        m.uc.hook_add(UC_HOOK_CODE,lambda *_: setattr(m,'hardware_valid',False),begin=address,end=address)
        m.tick()
        self.assertEqual(m.state()['fault'],'HARDWARE')
        self.assertEqual(m.outputs,[])

    def test_fsr_rx_budget_and_pressure_ping_motor_lease(self):
        m=Machine().arm(); m.now=499
        m.inject_rx(b'PING\n'*20)
        m.call('FSR_Poll')
        consumed=struct.unpack('<H',m.uc.mem_read(m.symbols['rx_tail'],2))[0]
        self.assertEqual(consumed,5)
        self.assertEqual(m.tx[-1],'#PONG,12')
        m.now=500; m.call('FSR_Poll')
        self.assertEqual(m.state()['fault'],'LEASE')
        m.inject_rx(b'x'*100); m.call('FSR_Poll')
        self.assertEqual(struct.unpack('<H',m.uc.mem_read(m.symbols['rx_tail'],2))[0],32)

    def test_fsr_uart_fault_forces_real_motor_off(self):
        m=Machine().arm()
        m.uc.mem_write(m.symbols['rx_fault'],b'\1'); m.call('FSR_Poll')
        self.assertEqual(m.state()['fault'],'UART_FAULT')
        self.assertEqual(m.word(TIM1+0x20),0)
        self.assertEqual(m.word(PE)&MODE_MASK,OUTPUT_MODE)

    def test_pressure_transmit_delay_checks_expired_lease_on_return(self):
        m=Machine().arm(); m.now=499
        m.transmit_hook=lambda item: setattr(item,'now',item.now+10)
        m.inject_rx(b'PING\n'); m.call('FSR_Poll')
        self.assertEqual(m.state()['armed'],0)
        self.assertEqual(m.state()['fault'],'LEASE')


class TuningTests(unittest.TestCase):
    def test_commit_is_explicit_atomic_validated_and_requires_new_sequence(self):
        m=Machine(); address=m.symbols['g_experiment_a_tune']
        initial=m.command('TUNE_STATE')[0]
        self.assertEqual(initial,'#TUNE,OASIS_A,v1,revision=0,yaw_max_mdeg=15000,'
            'pitch_max_mdeg=6000,yaw_rate_mdeg_s=2000,pitch_rate_mdeg_s=2000,'
            'full_scale_ay_mmps2=2942,pitch_scale_permille=1000')
        m.uc.mem_write(address,struct.pack('<i',12000)) # field edit without commit
        self.assertEqual(m.command('TUNE_STATE')[0],initial)
        m.uc.mem_write(address+24,struct.pack('<I',1))
        self.assertIn('revision=1,yaw_max_mdeg=12000',m.command('TUNE_STATE')[0])
        self.assertEqual(m.debug_snapshot()['tuning_result_code'],1)
        active=m.debug_snapshot()['tuning_active']
        m.tune(2,yaw_max_mdeg=8000,pitch_max_mdeg=6001)
        self.assertEqual(m.debug_snapshot()['tuning_result_code'],3)
        self.assertEqual(m.debug_snapshot()['tuning_revision'],1)
        self.assertEqual(m.debug_snapshot()['tuning_active'],active)
        m.tune(2,yaw_max_mdeg=8000) # same failed sequence cannot later apply
        self.assertEqual(m.debug_snapshot()['tuning_revision'],1)
        m.tune(1,yaw_max_mdeg=8000)
        self.assertEqual(m.debug_snapshot()['tuning_result_code'],4)
        m.tune(3,yaw_max_mdeg=8000)
        self.assertEqual(m.debug_snapshot()['tuning_revision'],3)
        self.assertEqual(m.debug_snapshot()['tuning_active'][0],8000)
        self.assertEqual(m.state()['pulses'],(0,)*4)

    def test_busy_commit_cannot_change_running_config_or_renew_lease(self):
        m=Machine().arm(); m.now=499
        m.tune(1,yaw_max_mdeg=8000)
        self.assertEqual(m.debug_snapshot()['tuning_result_code'],2)
        self.assertEqual(m.debug_snapshot()['tuning_revision'],0)
        self.assertEqual(m.debug_snapshot()['tuning_active'],TUNE_DEFAULTS)
        m.now=500; m.command('TUNE_STATE')
        self.assertEqual(m.state()['fault'],'LEASE')
        self.assertEqual(m.state()['pulses'],(0,)*4)
        m.tune(1,yaw_max_mdeg=8000) # a rejected armed request is not deferred
        self.assertEqual(m.debug_snapshot()['tuning_revision'],0)
        m.tune(2,yaw_max_mdeg=8000)
        self.assertEqual(m.debug_snapshot()['tuning_revision'],2)

    def test_arm_revision_race_and_legacy_syntax_cannot_enable_pwm(self):
        m=Machine().tune(1,yaw_max_mdeg=12000)
        self.assertEqual(m.command('FB_ARM,1'),['#ERR,0,INVALID'])
        self.assertEqual(m.command('FB_ARM,1,0'),['#ERR,0,TUNE_REVISION'])
        self.assertEqual(m.state()['pulses'],(0,)*4)
        m.arm(1,1)
        self.assertEqual(m.state()['pulses'],NEUTRAL)

    def test_each_tuning_integer_bound_rejects_whole_transaction(self):
        ranges=((100,CAD_YAW_MAX_MDEG),(0,CAD_PITCH_MAX_MDEG),(100,20000),
                (100,20000),(500,8000),(10,1000))
        for name,(low,high) in zip(TUNE_FIELDS,ranges):
            for value in (low-1,high+1,-2147483648,2147483647):
                with self.subTest(field=name,value=value):
                    m=Machine().tune(1,**{name:value})
                    self.assertEqual(m.debug_snapshot()['tuning_result_code'],3)
                    self.assertEqual(m.debug_snapshot()['tuning_active'],TUNE_DEFAULTS)

    def test_operating_cap_is_independent_of_cad_hard_cap(self):
        m=Machine().arm()
        self.assertTrue(pose_within_cad((16000,0,0,0)))
        self.assertEqual(m.command('FB_SET,1,1,16000,0,0,0'),['#ERR,1,TUNE_BOUNDS'])
        self.assertEqual(m.state()['fault'],'TUNE_BOUNDS')
        self.assertEqual(m.state()['pulses'],(0,)*4)

    def test_axis_rates_and_pwm_rate_both_hold_under_retargeting(self):
        m=Machine(math=True).tune(1,yaw_rate_mdeg_s=1000,pitch_rate_mdeg_s=500).arm()
        self.assertTrue(m.command('FB_SET,1,1,6000,6000,6000,-6000')[0].startswith('#ACK'))
        old=m.state()
        for i in range(1,81):
            if i%10==0: m.command(f'FB_PING,1,{i//10}')
            m.tick()
            new=m.state()
            self.assertEqual(new['armed'],1,new)
            self.assertTrue(pose_within_cad(new['pose']))
            for index,(a,b) in enumerate(zip(old['pose'],new['pose'])):
                self.assertLessEqual(abs(a-b),10 if index%2 else 20)
            self.assertTrue(all(abs(a-b)<=2 for a,b in zip(old['pulses'],new['pulses'])))
            old=new
        self.assertGreater(old['pose'][0],0)


class RegisterTests(unittest.TestCase):
    def test_watchdog_registers_and_only_complete_loop_feeds(self):
        m=Machine()
        self.assertEqual((m.word(IWDG+4),m.word(IWDG+8),m.word(IWDG+16)),(4,999,4095))
        self.assertEqual(m.word(DBG+0x4C)&1,0)
        self.assertEqual(m.word(DBG+0x54)&(1<<18),0)
        before=m.watchdog_refreshes
        m.command('STATE'); m.call('Control_Poll')
        self.assertEqual(m.watchdog_refreshes,before)
        m.call('Fallback_EndLoop'); self.assertEqual(m.watchdog_refreshes,before+1)

    def test_first_frame_zero_then_full_neutral_and_all_pin_af(self):
        m=Machine().arm()
        self.assertEqual(m.active,(0,)*4)
        self.assertEqual((m.word(TIM1+0x28),m.word(TIM1+0x2C)),(63,19999))
        for pin in (13,14,11,9):
            self.assertEqual((m.word(PE)>>(2*pin))&3,2)
            self.assertEqual((m.word(PE+0x24)>>(4*(pin-8)))&15,1)
        m.rollover(); self.assertEqual(m.active,NEUTRAL)

    def test_udis_batch_at_every_ccr_write_and_primask_restore(self):
        m=Machine().arm(); m.rollover(); m.word(TIM1+0x24,5000)
        new=(1798,1202,1202,1798)
        m.uc.mem_write(0x24060000,struct.pack('<4H',*new))
        m.force_rollover=True
        m.uc.reg_write(UC_ARM_REG_PRIMASK,1)
        self.assertEqual(m.call('Fallback_PwmStage',0x24060000),1)
        self.assertEqual(m.uc.reg_read(UC_ARM_REG_PRIMASK),1)
        self.assertTrue(all(frame==NEUTRAL or frame==(0,)*4 for frame in m.frames))
        m.force_rollover=False; m.rollover()
        self.assertEqual(m.active,new)
        self.assertFalse(any(address==TIM1+0x14 for address,_ in m.writes[-10:]))

    def test_stage_rejects_high_window_and_duplicate_frame(self):
        m=Machine().arm(); m.rollover()
        m.uc.mem_write(0x24060000,struct.pack('<4H',1798,1202,1202,1798))
        m.word(TIM1+0x24,1000)
        self.assertEqual(m.call('Fallback_PwmStage',0x24060000),0)
        m.word(TIM1+0x24,5000)
        self.assertEqual(m.call('Fallback_PwmStage',0x24060000),1)
        self.assertEqual(m.call('Fallback_PwmStage',0x24060000),0)

    def test_off_and_register_corruption_leave_all_outputs_low(self):
        for offset,value in ((0x20,0),(0x28,64),(0x2C,20000),(0x3C,1801)):
            m=Machine().arm(); m.word(TIM1+offset,value); m.call('Control_Poll')
            self.assertEqual(m.state()['fault'],'HARDWARE')
            self.assertEqual(m.word(PE)&MODE_MASK,OUTPUT_MODE)
            self.assertEqual(m.word(PE+0x14)&PINS,0)
            self.assertEqual(m.word(TIM1)&1,0)
            self.assertEqual(m.word(TIM1+0x20),0)
            self.assertEqual(m.word(TIM1+0x44)&0x8000,0)

    def test_no_mux_gpio_writes_by_pwm_and_arm(self):
        m=Machine().arm(); m.command('FB_OFF')
        self.assertFalse(any(0x58020000<=address<0x58021000 or
                            0x58021400<=address<0x58022000 for address,_ in m.writes))


class CadEnvelopeTests(unittest.TestCase):
    def test_hello_neutral_matches_arm_and_zero_pose(self):
        m=Machine(math=True)
        fields=dict(part.split('=',1) for part in m.command('CONTROL_HELLO')[0].split(',')[3:])
        self.assertEqual(fields['neutral_us'],':'.join(map(str,NEUTRAL)))
        self.assertEqual(fields['servo_range_deg'],'270')
        self.assertEqual(m.state()['pulses'],(0,)*4)
        m.arm()
        self.assertEqual(m.state()['pulses'],NEUTRAL)
        scratch=0x24060000
        m.uc.mem_write(scratch,struct.pack('<4i',0,0,0,0))
        self.assertEqual(m.call('Joint_PosePulses',scratch,scratch+32),1)
        self.assertEqual(struct.unpack('<4H',m.uc.mem_read(scratch+32,8)),NEUTRAL)

    def test_corrected_270_degree_gain_against_independent_golden_pulses(self):
        # Original R8 golden phi at yaw=1deg,pitch=0 is 2.372851deg.
        # 2000us/270deg gives 17.576674us -> 18us, not the 26us of 180deg.
        m=Machine(math=True)
        scratch=0x24060000
        m.uc.mem_write(scratch,struct.pack('<4i',1000,0,1000,0))
        self.assertEqual(m.call('Joint_PosePulses',scratch,scratch+32),1)
        self.assertEqual(struct.unpack('<4H',m.uc.mem_read(scratch+32,8)),(1782,1818,1218,1182))

    def test_actual_elf_crank_angles_and_mirrored_pulses_match_host(self):
        from experiment_a.kinematics import inverse_side,pose_to_pulses_us
        m=Machine(math=True)
        scratch=0x24060000
        for yaw,cap in CAD_CAP_POINTS:
            for pitch in (0,cap,-cap):
                m.uc.reg_write(UC_ARM_REG_D0,struct.unpack('<Q',struct.pack('<d',yaw/1000.))[0])
                m.uc.reg_write(UC_ARM_REG_D1,struct.unpack('<Q',struct.pack('<d',pitch/1000.))[0])
                self.assertEqual(m.call('Joint_CrankAngles',scratch),1)
                phi=struct.unpack('<2d',m.uc.mem_read(scratch,16))
                for actual,expected in zip(phi,inverse_side(yaw/1000.,pitch/1000.)):
                    self.assertAlmostEqual(actual,expected,places=9)
                pose=(yaw,pitch,yaw,-pitch)
                m.uc.mem_write(scratch,struct.pack('<4i',*pose))
                self.assertEqual(m.call('Joint_PosePulses',scratch,scratch+32),1)
                actual=struct.unpack('<4H',m.uc.mem_read(scratch+32,8))
                floats=pose_to_pulses_us(tuple(v/1000. for v in pose))
                expected=tuple(center+int((pulse-center)+(.5 if pulse>=center else -.5))
                               for pulse,center in zip(floats,NEUTRAL))
                self.assertEqual(actual,expected)

    def test_descriptor_matches_host_and_does_not_renew_motor_lease(self):
        m=Machine().arm()
        m.now=499
        self.assertEqual(m.command('CAD_INFO'),[CAD_INFO_LINE])
        self.assertEqual(m.state()['armed'],1)
        m.now=500
        self.assertEqual(m.command('CAD_INFO'),[CAD_INFO_LINE])
        self.assertEqual(m.state()['fault'],'LEASE')
        self.assertEqual(m.state()['pulses'],(0,)*4)

    def test_generated_header_and_profile_are_exact_current_host_inputs(self):
        from generate_fallback_geometry import artifacts
        for path,expected in artifacts().items():
            self.assertEqual(path.read_bytes(),expected,str(path))

    def test_actual_m7_integer_guard_matches_host_boundaries_and_interiors(self):
        import random
        m=Machine()
        scratch=0x24060000
        yaw_values={-2147483648,-1,0,1,CAD_YAW_MAX_MDEG-1,
                    CAD_YAW_MAX_MDEG,CAD_YAW_MAX_MDEG+1,2147483647}
        for (y0,_),(y1,_) in zip(CAD_CAP_POINTS,CAD_CAP_POINTS[1:]):
            yaw_values.update((y0-1,y0,y0+1,(y0+y1)//2,y1-1,y1,y1+1))
        rng=random.Random(2026)
        yaw_values.update(rng.randrange(0,CAD_YAW_MAX_MDEG+1) for _ in range(60))
        for yaw in sorted(yaw_values):
            cap=-1
            if 0<=yaw<=CAD_YAW_MAX_MDEG:
                for (y0,p0),(y1,p1) in zip(CAD_CAP_POINTS,CAD_CAP_POINTS[1:]):
                    if yaw<=y1:
                        cap=(p0*(y1-y0)+(p1-p0)*(yaw-y0))//(y1-y0)
                        break
            pitches={-2147483648,-CAD_PITCH_MAX_MDEG-1,0,CAD_PITCH_MAX_MDEG+1,2147483647}
            if cap>=0: pitches.update((-cap-1,-cap,cap,cap+1))
            else: self.assertFalse(0<=yaw<=CAD_YAW_MAX_MDEG)
            for pitch in sorted(pitches):
                for pose in ((yaw,pitch,0,0),(0,0,yaw,pitch)):
                    with self.subTest(pose=pose):
                        m.uc.mem_write(scratch,struct.pack('<4i',*pose))
                        self.assertEqual(bool(m.call('Fallback_CadPoseAllowed',scratch)),
                                         pose_within_cad(pose))
        self.assertEqual(m.call('Fallback_CadPoseAllowed',0),0)

    def test_all_22001_integer_yaws_at_positive_negative_boundary_and_one_beyond(self):
        m=Machine()
        scratch=0x24060000
        for yaw in range(CAD_YAW_MAX_MDEG+1):
            for (y0,p0),(y1,p1) in zip(CAD_CAP_POINTS,CAD_CAP_POINTS[1:]):
                if yaw<=y1:
                    cap=(p0*(y1-y0)+(p1-p0)*(yaw-y0))//(y1-y0)
                    break
            # Exercise both physical sides in the same query for each sign.
            for pitch,expected in ((cap,1),(-cap,1),(cap+1,0),(-cap-1,0)):
                m.uc.mem_write(scratch,struct.pack('<4i',yaw,pitch,yaw,-pitch))
                actual=m.call('Fallback_CadPoseAllowed',scratch)
                if actual!=expected:
                    self.fail(f'CAD mismatch yaw={yaw} pitch={pitch}: {actual} != {expected}')

    def test_cad_rejection_is_atomic_and_switches_all_outputs_off(self):
        targets=((0,1,0,0),(0,0,0,-1),(20000,6000,15000,0),
                 (0,0,CAD_YAW_MAX_MDEG+1,0))
        for target in targets:
            with self.subTest(target=target):
                m=Machine().arm()
                command='FB_SET,1,1,'+','.join(map(str,target))
                self.assertEqual(m.command(command),['#ERR,1,CAD_BOUNDS'])
                state=m.state()
                self.assertEqual(state['fault'],'CAD_BOUNDS')
                self.assertEqual(state['pulses'],(0,)*4)
                self.assertEqual(state['target'],(0,)*4)
                self.assertEqual(m.word(TIM1+0x20),0)

    def test_waypoint_quantization_only_clamps_one_mdeg_inward(self):
        m=Machine()
        scratch=0x24060000
        for original,expected in (((20000,2001,20000,-2001),(20000,2000,20000,-2000)),
                                  ((0,1,0,-1),(0,0,0,0))):
            m.uc.mem_write(scratch,struct.pack('<4i',*original))
            self.assertEqual(m.call('Fallback_CadRoundInward',scratch),1)
            self.assertEqual(struct.unpack('<4i',m.uc.mem_read(scratch,16)),expected)
        for original in ((20000,2002,0,0),(0,0,20000,-2002),(-1,0,0,0),
                         (0,-2147483648,0,0),(0,2147483647,0,0)):
            m.uc.mem_write(scratch,struct.pack('<4i',*original))
            self.assertEqual(m.call('Fallback_CadRoundInward',scratch),0)

    def test_due_update_cannot_bypass_cad_guard_by_corrupted_target(self):
        m=Machine(math=True).arm()
        m.command('FB_SET,1,1,1000,0,0,0')
        m.uc.mem_write(m.symbols['target'],struct.pack('<4i',20000,6000,0,0))
        m.tick()
        self.assertEqual(m.state()['fault'],'CAD_BOUNDS')
        self.assertEqual(m.outputs,[])

    def test_intermediate_math_result_is_checked_before_pwm_stage(self):
        m=Machine(math=True).arm()
        m.command('FB_SET,1,1,1000,0,0,0')
        address=m.symbols['Joint_PoseStep']&~1
        def corrupt_next_at_return(uc,unused_address,unused_size,user):
            pointer=uc.reg_read(UC_ARM_REG_R1)
            return_address=uc.reg_read(UC_ARM_REG_LR)&~1
            def corrupt(*unused):
                m.uc.mem_write(pointer,struct.pack('<4i',20000,6000,0,0))
            m.uc.hook_add(UC_HOOK_CODE,corrupt,begin=return_address,end=return_address)
        m.uc.hook_add(UC_HOOK_CODE,corrupt_next_at_return,begin=address,end=address)
        m.tick()
        self.assertEqual(m.state()['fault'],'CAD_PATH')
        self.assertEqual(m.outputs,[])


if __name__=='__main__': unittest.main(verbosity=2)
