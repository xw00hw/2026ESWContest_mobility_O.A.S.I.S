"""Optional same-Pi, preprocessed pose input. Imports never open a camera/socket.

The wire contract is oasis.pose.v1, not an unmodified DAQ features row. Angles
and lean score are relative to a calibrated neutral pose. Capture clocks are
explicit; receipt time never makes an old frame fresh. Metric body estimation
is not part of this interface.
"""
from collections import Counter
import ipaddress
import json
import math
import socket
import time

from .sources import NoPose

SCHEMA = 'oasis.pose.v1'
MAX_BYTES, MAX_DATAGRAMS = 4096, 32
FIELDS = frozenset(('schema', 'source_id', 'stream_id', 'seq', 'capture_clock', 'capture_ns',
                    'torso_lean_deg', 'lean_score', 'pose_confidence', 'vision_valid', 'neutral_valid'))


class PoseInputError(ValueError):
    pass


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PoseInputError('invalid_' + name)
    try:
        value = float(value)
    except (ValueError, OverflowError):
        raise PoseInputError('invalid_' + name) from None
    if not math.isfinite(value):
        raise PoseInputError('nonfinite_' + name)
    return value


def validate_config(cfg):
    """Validate before opening any input. Disabled input has no socket side effect."""
    if (cfg.get('profile') or {}).get('from_camera', False):
        raise ValueError('profile.from_camera requires a calibrated metric-body interface; keep it false')
    pose = cfg.get('pose') or {}
    source = pose.get('source', 'none')
    if source not in ('none', 'udp'):
        raise ValueError('pose.source must be none or udp')
    if source == 'none':
        return dict(source='none')
    source_id = pose.get('source_id')
    if not isinstance(source_id, str) or not source_id.strip() or len(source_id) > 128:
        raise ValueError('pose.source_id must identify the expected local pose producer')
    port = pose.get('udp_port', 47802)
    if type(port) is not int or not 0 <= port <= 65535:
        raise ValueError('pose.udp_port must be an integer in 0..65535')
    stale = _number(pose.get('stale_s', .3), 'stale_s')
    confidence = _number(pose.get('min_confidence', .5), 'min_confidence')
    if not 0 < stale <= 1 or not 0 <= confidence <= 1:
        raise ValueError('pose.stale_s must be in (0,1]; min_confidence in [0,1]')
    if pose.get('lean_sign', 1) not in (-1, 1) or isinstance(pose.get('lean_sign', 1), bool):
        raise ValueError('pose.lean_sign must be +1 or -1')
    return dict(source='udp', source_id=source_id, udp_port=port, stale_s=stale, min_confidence=confidence)


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise PoseInputError('duplicate_field')
        result[key] = value
    return result


def _nonfinite(_value):
    raise PoseInputError('nonfinite_json')


class UdpPose:
    """Bounded nonblocking loopback receiver; invalid/stale samples become unavailable.

    Inject transport/clock providers for offline tests. A producer restart must
    use a new stream_id. Within a stream, both seq and capture time increase.
    Clock providers return integer ns in the named, same-host clock domain.
    """
    def __init__(self, options, clock_ns, journal, *, transport=None, clocks=None):
        self.options, self.clock_ns, self.journal = dict(options), clock_ns, journal
        self.clocks = dict(clocks) if clocks is not None else {'monotonic': clock_ns}
        if clocks is None and hasattr(time, 'CLOCK_MONOTONIC_RAW'):
            self.clocks['monotonic_raw'] = lambda: time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
        self.last = None
        self._stream = self._seq = self._capture = self._capture_clock = None
        self._closed = False
        self.accepted = self.received = 0
        self.reasons = Counter()
        self.last_reason = 'no_samples'
        self.s = transport
        try:
            if self.s is None:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.s.bind(('127.0.0.1', options['udp_port']))
                self.s.setblocking(False)
            self.journal.record('pose_source_enabled', host_monotonic_ns=self.clock_ns(),
                                schema=SCHEMA, source_id=options['source_id'], listen_host='127.0.0.1',
                                udp_port=options['udp_port'], metric_body_enabled=False)
        except BaseException:
            if self.s is not None:
                self.s.close()
            raise

    def _reject(self, reason, *, byte_count=None):
        self.last = None
        self.last_reason = reason
        self.reasons[reason] += 1
        self.journal.record('pose_rejected', host_monotonic_ns=self.clock_ns(),
                            reason=reason, byte_count=byte_count)

    def _age(self, packet):
        clock = packet['capture_clock']
        if clock not in ('monotonic', 'monotonic_raw') or clock not in self.clocks:
            raise PoseInputError('capture_clock_unavailable')
        try:
            now = self.clocks[clock]()
        except (OSError, ValueError):
            raise PoseInputError('capture_clock_unavailable') from None
        if type(now) is not int or now < 0:
            raise PoseInputError('invalid_local_clock')
        age_ns = now - packet['capture_ns']
        if age_ns < 0:
            raise PoseInputError('future_capture')
        age = age_ns / 1e9
        if age >= self.options['stale_s']:
            raise PoseInputError('stale_capture')
        return age

    def _parse(self, data, peer):
        try:
            if not ipaddress.ip_address(peer[0]).is_loopback:
                raise PoseInputError('non_loopback_sender')
        except (ValueError, IndexError):
            raise PoseInputError('non_loopback_sender') from None
        if len(data) > MAX_BYTES:
            raise PoseInputError('oversize_datagram')
        try:
            packet = json.loads(data, object_pairs_hook=_object, parse_constant=_nonfinite)
        except PoseInputError:
            raise
        except (ValueError, UnicodeError, RecursionError):
            raise PoseInputError('invalid_json') from None
        if not isinstance(packet, dict) or set(packet) != FIELDS:
            raise PoseInputError('invalid_fields')
        if packet['schema'] != SCHEMA:
            raise PoseInputError('schema_mismatch')
        if packet['source_id'] != self.options['source_id']:
            raise PoseInputError('source_mismatch')
        if not isinstance(packet['stream_id'], str) or not packet['stream_id'].strip() or len(packet['stream_id']) > 128:
            raise PoseInputError('invalid_stream_id')
        if type(packet['seq']) is not int or not 0 <= packet['seq'] <= 2**63-1:
            raise PoseInputError('invalid_sequence')
        if type(packet['capture_ns']) is not int or not 0 <= packet['capture_ns'] <= 2**63-1:
            raise PoseInputError('invalid_capture_timestamp')
        if not isinstance(packet['capture_clock'], str):
            raise PoseInputError('capture_clock_unavailable')
        age = self._age(packet)
        if packet['stream_id'] == self._stream:
            if packet['capture_clock'] != self._capture_clock:
                raise PoseInputError('capture_clock_changed')
            if packet['seq'] <= self._seq:
                raise PoseInputError('sequence_not_increasing')
            if packet['capture_ns'] <= self._capture:
                raise PoseInputError('capture_not_increasing')
        elif packet['capture_clock'] == self._capture_clock and self._capture is not None and packet['capture_ns'] <= self._capture:
            raise PoseInputError('capture_not_increasing')
        if packet['vision_valid'] is not True or packet['neutral_valid'] is not True:
            raise PoseInputError('invalid_vision_or_neutral')
        for field in ('torso_lean_deg', 'lean_score', 'pose_confidence'):
            packet[field] = _number(packet[field], field)
        if not 0 <= packet['pose_confidence'] <= 1:
            raise PoseInputError('invalid_confidence')
        if packet['pose_confidence'] < self.options['min_confidence']:
            raise PoseInputError('low_confidence')
        if abs(packet['torso_lean_deg']) > 90 or abs(packet['lean_score']) > 4:
            raise PoseInputError('pose_out_of_range')
        return packet, age

    def poll(self):
        if self._closed:
            return
        # One additional receive detects overflow without an unbounded drain.
        for index in range(MAX_DATAGRAMS+1):
            try:
                data, peer = self.s.recvfrom(MAX_BYTES+1)
            except BlockingIOError:
                break
            except OSError:
                self._reject('socket_error')
                break
            self.received += 1
            if index == MAX_DATAGRAMS:
                self._reject('datagram_backlog', byte_count=len(data))
                break
            try:
                packet, age = self._parse(data, peer)
            except PoseInputError as exc:
                self._reject(str(exc), byte_count=len(data))
                continue
            self.journal.record('pose_sample', host_monotonic_ns=self.clock_ns(),
                                capture_age_s=age, packet=packet)
            self.last = packet
            self._stream, self._seq = packet['stream_id'], packet['seq']
            self._capture, self._capture_clock = packet['capture_ns'], packet['capture_clock']
            self.accepted += 1
            self.last_reason = 'ok'

    def latest(self):
        self.poll()
        if self._closed or self.last is None:
            return None, 1e9
        try:
            age = self._age(self.last)
        except PoseInputError as exc:
            self._reject(str(exc))
            return None, 1e9
        return dict(self.last), age

    def summary(self):
        return dict(source='udp', enabled=True, schema=SCHEMA, source_id=self.options['source_id'],
                    listen_host='127.0.0.1', udp_port=self.options['udp_port'],
                    capture_clock=self._capture_clock, stream_id=self._stream, last_sequence=self._seq,
                    received=self.received, accepted=self.accepted, reasons=dict(self.reasons),
                    last_reason=self.last_reason, closed=self._closed, metric_body_enabled=False)

    def close(self):
        if not self._closed:
            self._closed = True
            self.last = None
            self.s.close()


def make_pose_input(cfg, clock_ns, journal, *, allow_udp=True, transport=None, clocks=None):
    options = validate_config(cfg)
    if options['source'] == 'none':
        return NoPose()
    if not allow_udp and transport is None:
        raise ValueError('SIL pose input requires an injected transport; no live pose socket is opened')
    return UdpPose(options, clock_ns, journal, transport=transport, clocks=clocks)


def close_pose(source):
    close = getattr(source, 'close', None)
    if close is not None:
        close()


def pose_summary(source):
    summary = getattr(source, 'summary', None)
    return summary() if summary else dict(source='none', enabled=False, metric_body_enabled=False)
