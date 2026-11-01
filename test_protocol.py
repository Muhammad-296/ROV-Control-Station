"""Static/behavioral GUI protocol checks using AST; no Qt or real hardware required."""
import ast
import time
import textwrap
from pathlib import Path
src = Path(__file__).with_name('ROV_Control_Station_Integrated.py').read_text('utf-8')
module = ast.parse(src)
main = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'MainWindow')
methods = ('_motor_test_allowed','_send_test_packet','_individual_hold',
           '_individual_release','_individual_motor_command','_release_all',
           '_individual_stop','_record_test_event','_update_motor_test_buttons')
ns = {'time': time, 'MOTOR_NAMES': ('FL','FR','RL','RR','Vertical L','Vertical R')}
for name in methods:
    method = next(n for n in main.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(textwrap.dedent(ast.get_source_segment(src, method)), ns)

class Widget:
    def __init__(self, value=15): self.v=value
    def value(self): return self.v
    def setEnabled(self, _): pass
    def setText(self, _): pass
    def setDown(self, _): pass
    def setProperty(self, *_): pass
    def style(self): return self
    def unpolish(self, *_): pass
    def polish(self, *_): pass

class Link:
    def __init__(self): self.lines=[]
    def is_open(self): return True
    def send(self,line): self.lines.append(line); return True

class Fake:
    def __init__(self):
        self.link=Link();self.ready=True;self.armed=True;self.firmware_state='ARMED'
        self.desired_mode='INDIVIDUAL';self.firmware_mode='INDIVIDUAL'
        self.motor_held_index=None;self.motor_held_dir=0;self.motor_held_source=None;self.motor_held_power=0
        self.motor_selected=0;self.motor_speed_sliders=[Widget(x) for x in (10,15,20,25,30,35)]
        self.motor_test_reverse=Widget();self.motor_invert_boxes=[Widget() for _ in range(6)]
        self.motor_keyboard_hint=Widget();self.motor_key_buttons=[Widget() for _ in range(6)]
        self.keys_down=set();self.hold_buttons={};self.test_sequence=0
        self.test_first_sent=0;self.last_test_ack=0;self.test_tx_count=0
        self.last_send=0;self.test_log=[];self.pending_arm=False;self.records=[]
    def log(self,*args):self.records.append(args)
    def _update_arm_ui(self): pass
    def emergency_stop(self): raise AssertionError('Unexpected emergency stop')
    def set_armed(self, desired):
        assert desired is False
        self.armed=False
        self.link.send('SYS,DISARM')
for name in methods: setattr(Fake,name,ns[name])

a=Fake()
a._individual_hold(0,1,'key:0')
assert a.link.lines[-1]=='MT,1,0,10'
a._individual_hold(2,-1,'key:2')
assert a.link.lines[-1]=='MT,2,2,-20'
a._individual_release(0,'key:0')
assert a.motor_held_index==2, 'releasing old key must not stop newly active motor'
a._individual_release(2,'key:2')
assert a.link.lines[-1]=='MT,3,-1,0'
assert a._individual_motor_command()==[0]*6
# normal focus-loss cleanup immediately sends zero
b=Fake();b._individual_hold(4,1,'key:4');b._release_all()
assert b.link.lines[-1]=='MT,2,-1,0'
# stop disarms the firmware
c=Fake();c._individual_hold(5,1,'mouse');c._individual_stop()
assert c.link.lines[-1]=='SYS,DISARM' and not c.armed
# cannot send while disarmed / wrong mode
d=Fake();d.armed=False
assert not d._send_test_packet([10,0,0,0,0,0])
d.armed=True;d.firmware_mode='DRIVE'
assert not d._send_test_packet([10,0,0,0,0,0])
# never allow multi motor request
e=Fake();assert not e._send_test_packet([10,10,0,0,0,0])
assert e.link.lines==[]
assert 'SYS,HELLO,5' in src and 'MT:ACK,' in src
assert 'QMainWindow(self)' in src
print('PASS: direct test key packets, immediate release, focus loss, mode isolation, stop/disarm, V5 handshake')
