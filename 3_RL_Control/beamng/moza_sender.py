"""Optional read-only MOZA R3 X-axis sender; never acquires a wheel or sends FFB.

Windows WinMM queries can run alongside BeamNG. Times are PC monotonic QUERY
times, not USB hardware capture times. Raw normalized-axis counts are kept
separate from BeamNG steering; physical degrees and game gain belong to the
receiving calibration. Importing this module opens no device or socket.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import sys
import time
import uuid

SCHEMA = 'oasis.moza.v1'
VID, PID = 0x346e, 0x0005
AXIS_MIN, AXIS_MAX = 0, 65535


@dataclass(frozen=True)
class WheelRead:
    valid: bool
    axis_raw: int | None
    reason: str
    observed: dict | None = None


class _WinMMApi:
    """The only imported native functions are two read-only joystick queries."""
    def __init__(self):
        if os.name != 'nt':
            raise RuntimeError('WinMM wheel input is available only on Windows')
        import ctypes as c
        from ctypes import wintypes as w
        class Caps(c.Structure):
            _fields_ = [('mid',w.WORD),('pid',w.WORD),('name',w.WCHAR*32)] + [
                (name,w.UINT) for name in
                'xmin xmax ymin ymax zmin zmax buttons periodmin periodmax rmin rmax umin umax vmin vmax caps maxaxes axes maxbuttons'.split()
            ] + [('regkey',w.WCHAR*32),('oem',w.WCHAR*260)]
        class Pos(c.Structure):
            _fields_ = [(name,w.DWORD) for name in
                        'size flags x y z r u v buttons button pov reserved1 reserved2'.split()]
        self.c, self.Caps, self.Pos = c, Caps, Pos
        self.dll = c.WinDLL('winmm')
        self.dll.joyGetDevCapsW.argtypes = [c.c_size_t,c.POINTER(Caps),w.UINT]
        self.dll.joyGetDevCapsW.restype = w.UINT
        self.dll.joyGetPosEx.argtypes = [w.UINT,c.POINTER(Pos)]
        self.dll.joyGetPosEx.restype = w.UINT

    def get_caps(self, joy_id):
        caps = self.Caps()
        code = self.dll.joyGetDevCapsW(joy_id,self.c.byref(caps),self.c.sizeof(caps))
        return code, {name:getattr(caps,name) for name in ('mid','pid','xmin','xmax','axes','name')}

    def get_x(self, joy_id):
        pos = self.Pos(size=self.c.sizeof(self.Pos),flags=0x1)  # JOY_RETURNX only
        code = self.dll.joyGetPosEx(joy_id,self.c.byref(pos))
        return code, int(pos.x)


class WinMMReader:
    """Exact identity/range checked every poll; errors never select another wheel.

    Injectable API: get_caps(id)->(return_code,dict), get_x(id)->(return_code,int).
    A disconnected device can recover at the same explicitly selected index.
    """
    def __init__(self, api=None):
        self.api = _WinMMApi() if api is None else api

    def read(self, joy_id=0):
        if type(joy_id) is not int or not 0 <= joy_id <= 15:
            raise ValueError('joy_id must be an integer in 0..15')
        try:
            code, caps = self.api.get_caps(joy_id)
            if code:
                return WheelRead(False,None,f'caps_error:{code}')
            if caps.get('mid') != VID or caps.get('pid') != PID:
                return WheelRead(False,None,'device_mismatch',caps)
            if caps.get('xmin') != AXIS_MIN or caps.get('xmax') != AXIS_MAX:
                return WheelRead(False,None,'axis_range_mismatch',caps)
            code, raw = self.api.get_x(joy_id)
            if code:
                return WheelRead(False,None,f'query_error:{code}',caps)
            if type(raw) is not int or not AXIS_MIN <= raw <= AXIS_MAX:
                return WheelRead(False,None,'axis_value_invalid',caps)
            return WheelRead(True,raw,'',caps)
        except (OSError, ValueError, TypeError) as exc:
            return WheelRead(False,None,'query_exception:'+type(exc).__name__)


def _identifier(value, name):
    if not isinstance(value,str) or not value.strip() or len(value)>128 or any(ord(c)<32 for c in value):
        raise ValueError('invalid '+name)
    return value


def make_packet(read, *, source_id, stream_id, seq, joy_id, pc_query_ns, pc_send_ns):
    for value,name in ((pc_query_ns,'pc_query_ns'),(pc_send_ns,'pc_send_ns'),(seq,'seq')):
        if type(value) is not int or not 0 < value <= 2**63-1:
            raise ValueError(name+' must be a positive int64')
    if pc_send_ns < pc_query_ns:
        raise ValueError('PC clock regressed during query')
    if type(joy_id) is not int or not 0 <= joy_id <= 15:
        raise ValueError('invalid joy_id')
    if not isinstance(read,WheelRead) or type(read.valid) is not bool:
        raise ValueError('expected WheelRead with explicit validity')
    if read.valid:
        if type(read.axis_raw) is not int or not AXIS_MIN <= read.axis_raw <= AXIS_MAX or read.reason != '':
            raise ValueError('invalid valid-wheel sample')
    elif read.axis_raw is not None or not isinstance(read.reason,str) or not read.reason or len(read.reason)>128:
        raise ValueError('invalid unavailable-wheel sample')
    return dict(schema=SCHEMA,source_id=_identifier(source_id,'source_id'),
                stream_id=_identifier(stream_id,'stream_id'),seq=seq,
                device_key=f'winmm:{VID:04x}:{PID:04x}:{joy_id}:x',
                pc_query_ns=pc_query_ns,pc_send_ns=pc_send_ns,valid=read.valid,
                axis_raw=read.axis_raw,axis_min=AXIS_MIN,axis_max=AXIS_MAX,reason=read.reason)


def _destination(host, port):
    # getaddrinfo retains IPv6 scope IDs, including Windows link-local %12.
    entries = socket.getaddrinfo(host,port,type=socket.SOCK_DGRAM)
    if not entries:
        raise ValueError('no UDP destination')
    family,_,_,_,address = entries[0]
    return family,address


def serve(pi, *, log_path, port=64258, seconds=None, joy_id=0, hz=100., source_id='moza-r3',
          reader=None, socket_factory=socket.socket, destination_resolver=_destination,
          clock_ns=time.perf_counter_ns, sleep=time.sleep, stop=None, log=print):
    """Send bounded nonblocking raw samples; missing wheel sends valid=false.

    Every produced packet is logged, including send drops; summary distinguishes
    attempts from successful sends. Existing logs/metadata are never overwritten.
    Logging failure stops this optional producer. No motor interface is present.
    """
    if isinstance(hz,bool) or not isinstance(hz,(int,float)) or not math.isfinite(hz) or not 1 <= hz <= 250:
        raise ValueError('hz must be finite in 1..250')
    if seconds is not None and (isinstance(seconds,bool) or not isinstance(seconds,(int,float)) or not math.isfinite(seconds) or seconds <= 0):
        raise ValueError('seconds must be finite and positive')
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('port must be in 1..65535')
    if type(joy_id) is not int or not 0 <= joy_id <= 15:
        raise ValueError('joy_id must be in 0..15')
    _identifier(source_id,'source_id')
    path = Path(log_path)
    meta_path, summary_path = Path(str(path)+'.meta.json'), Path(str(path)+'.summary.json')
    if any(p.exists() for p in (path,meta_path,summary_path)):
        raise FileExistsError('choose a new immutable wheel-log path')
    reader = WinMMReader() if reader is None else reader
    family,address = destination_resolver(pi,port)
    stream_id = uuid.uuid4().hex
    metadata = dict(schema='oasis.moza.log.v1',packet_schema=SCHEMA,source_id=source_id,stream_id=stream_id,
                    device_key=f'winmm:{VID:04x}:{PID:04x}:{joy_id}:x',hz_requested=hz,
                    axis_min=AXIS_MIN,axis_max=AXIS_MAX,physical_degrees_in_packet=False,
                    timestamp_semantics='PC perf_counter_ns before query / before serialization and UDP send; not hardware capture time',
                    raw_log_semantics='all produced packet attempts; see summary for UDP send drops',
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    hardware_output_functions_used=False)
    stats = dict(samples=0,valid=0,invalid=0,sent=0,send_drops=0,error=None)
    reasons,tx,owns_log = Counter(),None,False
    period_ns = round(1e9/hz)
    start = clock_ns(); deadline = None if seconds is None else start+round(seconds*1e9)
    next_query, next_status = start,start+5_000_000_000
    try:
        with path.open('x',encoding='utf-8',newline='\n') as raw_log:
            owns_log=True
            with meta_path.open('x',encoding='utf-8') as f:
                json.dump(metadata,f,indent=2,allow_nan=False)
            tx=socket_factory(family,socket.SOCK_DGRAM); tx.setblocking(False)
            log(f'MOZA read-only sender {metadata["device_key"]} -> {address}; raw counts, {hz:g} Hz')
            while stop is None or not stop():
                now=clock_ns()
                if deadline is not None and now>=deadline: break
                if now<next_query:
                    sleep(min(next_query-now,(deadline-now) if deadline else next_query-now)/1e9)
                    continue
                query_ns=clock_ns()
                read=reader.read(joy_id)
                send_ns=clock_ns()
                packet=make_packet(read,source_id=source_id,stream_id=stream_id,seq=stats['samples']+1,
                                   joy_id=joy_id,pc_query_ns=query_ns,pc_send_ns=send_ns)
                data=json.dumps(packet,separators=(',',':'),allow_nan=False).encode('utf-8')
                stats['samples']+=1; stats['valid' if read.valid else 'invalid']+=1
                reasons[read.reason or 'ok']+=1
                try:
                    if tx.sendto(data,address)!=len(data): raise OSError('partial UDP send')
                    stats['sent']+=1
                except (BlockingIOError,OSError): stats['send_drops']+=1
                raw_log.write(data.decode('utf-8')+'\n'); raw_log.flush()
                now=clock_ns()
                if now>=next_status:
                    log(f'MOZA samples={stats["samples"]} valid={stats["valid"]} unavailable={stats["invalid"]} send_drops={stats["send_drops"]}')
                    next_status=now+5_000_000_000
                # Skip missed periods; never burst a backlog of stale samples.
                next_query=max(next_query+period_ns,now)
    except KeyboardInterrupt:
        stats['error']='operator_stop'
    except BaseException as exc:
        stats['error']=type(exc).__name__+': '+str(exc)
        raise
    finally:
        if tx is not None: tx.close()
        stats.update(stream_id=stream_id,reasons=dict(reasons),elapsed_s=max(0,clock_ns()-start)/1e9,
                     raw_log=str(path),hardware_output_functions_used=False)
        if owns_log:
            with summary_path.open('x',encoding='utf-8') as f: json.dump(stats,f,indent=2,allow_nan=False)
    return stats


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi',required=True)
    ap.add_argument('--port',type=int,default=64258)
    ap.add_argument('--seconds',type=float)
    ap.add_argument('--joy-id',type=int,default=0)
    ap.add_argument('--hz',type=float,default=100)
    ap.add_argument('--source-id',default='moza-r3')
    ap.add_argument('--log',required=True,type=Path,help='new JSONL path; parent directory must exist')
    a=ap.parse_args(argv)
    result=serve(a.pi,port=a.port,seconds=a.seconds,joy_id=a.joy_id,hz=a.hz,
                 source_id=a.source_id,log_path=a.log)
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__': sys.exit(main())
