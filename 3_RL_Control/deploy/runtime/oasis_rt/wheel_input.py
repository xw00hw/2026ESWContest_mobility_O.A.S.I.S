"""Optional Windows WinMM MOZA input; no socket is opened by import or default.

PC timestamps are HOST QUERY times, not USB capture times. PC and Pi clocks
cannot be compared directly. Age includes query-to-send, local residence and
delay relative to the minimum observed receive-minus-send offset per stream.
The initial/constant network delay is unknown; this is not a synchronized
end-to-end latency measurement. Calibration is mandatory for control use.
The nominal sender rate is 100 Hz. Raw 0..65535 maps to norm -1..+1;
physical degrees are norm*physical_span_deg/2. Signed game gain is degrees
at full normalized travel, independent of that physical span. An absent,
invalid, stale or uncalibrated source returns unavailable, not a motor fault.
"""
from collections import Counter
import ipaddress
import json
import math
import socket

SCHEMA = 'oasis.moza.v1'
DEVICE_KEY = 'winmm:346e:0005:0:x'
MAX_BYTES, MAX_DATAGRAMS, MAX_STREAMS = 4096, 32, 64
FIELDS = frozenset(('schema', 'source_id', 'stream_id', 'seq', 'device_key',
                    'pc_query_ns', 'pc_send_ns', 'valid', 'axis_raw',
                    'axis_min', 'axis_max', 'reason'))


class WheelInputError(ValueError):
    pass


class WheelBindError(OSError):
    """Only transport initialization errors may degrade to optional NoWheel."""


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('wheel.' + name + ' must be a finite number')
    try:
        value = float(value)
    except (ValueError, OverflowError):
        raise ValueError('wheel.' + name + ' must be a finite number') from None
    if not math.isfinite(value):
        raise ValueError('wheel.' + name + ' must be a finite number')
    return value


def _text(value, name, limit=128):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError('wheel.' + name + ' must be a nonempty bounded string')
    return value


def _ip(host):
    # Scope selects the local NIC; canonical remote IP is the allowlist identity.
    return str(ipaddress.ip_address(host.split('%', 1)[0]))


def validate_config(cfg):
    options = cfg.get('wheel', {})
    if options is None:
        options = {}
    if not isinstance(options, dict):
        raise ValueError('wheel must be a mapping')
    if type(options.get('priority_prediction', False)) is not bool:
        raise ValueError('wheel.priority_prediction must be boolean')
    source = options.get('source', 'none')
    if source not in ('none', 'udp'):
        raise ValueError('wheel.source must be none or udp')
    if source == 'none':
        return dict(source='none')
    source_id = _text(options.get('source_id', 'moza-r3'), 'source_id')
    device_key = _text(options.get('device_key', DEVICE_KEY), 'device_key')
    listen = options.get('listen', ['::', 64258])
    if not isinstance(listen, (tuple, list)) or len(listen) != 2:
        raise ValueError('wheel.listen must be [numeric_host, port]')
    host, port = listen
    if not isinstance(host, str) or type(port) is not int or not 0 <= port <= 65535:
        raise ValueError('wheel.listen must be [numeric_host, port]')
    _ip(host)
    allow = options.get('allow_from')
    if not isinstance(allow, (list, tuple)) or not allow or len(allow) > 16:
        raise ValueError('wheel.allow_from requires 1..16 explicit PC IP addresses')
    if any(not isinstance(value, str) for value in allow):
        raise ValueError('wheel.allow_from must contain IP strings')
    allow = frozenset(_ip(value) for value in allow)
    span = _number(options.get('physical_span_deg', 450), 'physical_span_deg')
    fresh = _number(options.get('fresh_ms', 100), 'fresh_ms')
    pc_age = _number(options.get('max_pc_age_ms', 50), 'max_pc_age_ms')
    if not 0 < span <= 2160 or not 0 < pc_age <= fresh <= 1000:
        raise ValueError('invalid wheel span/freshness bounds')
    calibration = options.get('calibration', {})
    if calibration is None:
        calibration = {}
    if not isinstance(calibration, dict) or type(calibration.get('confirmed', False)) is not bool:
        raise ValueError('wheel.calibration.confirmed must be boolean')
    confirmed = calibration.get('confirmed', False)
    vehicle_id = calibration.get('beamng_vehicle_id')
    if vehicle_id is not None:
        vehicle_id = _text(vehicle_id, 'calibration.beamng_vehicle_id')
    gain = calibration.get('game_gain_deg')
    if gain is not None:
        gain = _number(gain, 'calibration.game_gain_deg')
    offset = _number(calibration.get('game_offset_deg', 0), 'calibration.game_offset_deg')
    if abs(offset) > 10 or (gain is not None and not 0 < abs(gain) <= 1080-abs(offset)):
        raise ValueError('invalid wheel game gain/offset bounds')
    if confirmed and gain is None:
        raise ValueError('confirmed wheel calibration requires game_gain_deg')
    return dict(source='udp', source_id=source_id, device_key=device_key,
                priority_prediction=options.get('priority_prediction', False),
                listen=(host, port), allow_from=allow, physical_span_deg=span,
                fresh_ms=fresh, max_pc_age_ms=pc_age,
                calibration=dict(confirmed=confirmed, game_gain_deg=gain, game_offset_deg=offset,
                                 beamng_vehicle_id=vehicle_id))


class NoWheel:
    def __init__(self, reason='disabled'):
        self.reason, self.closed = reason, False

    def poll(self):
        return None

    def latest(self):
        return None, 1e9

    def summary(self):
        return dict(source='none', enabled=False, reason=self.reason, closed=self.closed)

    def close(self):
        self.closed = True


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise WheelInputError('duplicate_field')
        result[key] = value
    return result


def _nonfinite(_value):
    raise WheelInputError('nonfinite_json')


def _open_socket(listen):
    sock = None
    try:
        host, port = listen
        if '%' in host:
            address, zone = host.rsplit('%', 1)
            if not zone.isdecimal():
                host = f'{address}%{socket.if_nametoindex(zone)}'
        family, _, _, _, address = socket.getaddrinfo(
            host, port, socket.AF_UNSPEC, socket.SOCK_DGRAM, socket.IPPROTO_UDP)[0]
        sock = socket.socket(family, socket.SOCK_DGRAM)
        if family == socket.AF_INET6:
            sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        sock.bind(address)
        sock.setblocking(False)
        return sock
    except OSError as exc:
        if sock is not None:
            sock.close()
        raise WheelBindError(str(exc)) from exc


class UdpWheel:
    def __init__(self, options, clock_ns, journal, *, transport=None):
        self.options, self.clock_ns, self.journal = dict(options), clock_ns, journal
        self.s = transport if transport is not None else _open_socket(options['listen'])
        self.closed, self.last = False, None
        self.received = self.validated = self.accepted = 0
        self.reasons, self.last_reason = Counter(), 'no_samples'
        self._stream = self._seq = self._query = self._send = None
        self._offset_min = self._last_now = None
        # No eviction: after 64 restarts refuse new streams until the receiver is
        # restarted, rather than accepting a replay of an evicted retired ID.
        self._seen_streams = set()
        try:
            self.journal.record('wheel_source_enabled', host_monotonic_ns=self._now(),
                                source_id=options['source_id'], device_key=options['device_key'],
                                listen=list(options['listen']), allow_from=sorted(options['allow_from']),
                                calibration=dict(options['calibration']),
                                timestamp_kind='pc_host_query_not_usb_capture',
                                absolute_network_latency_known=False)
        except BaseException:
            self.s.close()
            raise

    def _now(self):
        now = self.clock_ns()
        if type(now) is not int or now < 0:
            raise WheelInputError('invalid_local_clock')
        if self._last_now is not None and now < self._last_now:
            raise WheelInputError('local_clock_regressed')
        self._last_now = now
        return now

    def _reject(self, reason, *, now=None, byte_count=None):
        self.last = None
        self.last_reason = reason
        self.reasons[reason] += 1
        self.journal.record('wheel_rejected', host_monotonic_ns=now,
                            reason=reason, byte_count=byte_count)

    def _parse(self, data, peer):
        try:
            if _ip(peer[0]) not in self.options['allow_from']:
                raise WheelInputError('foreign_sender')
        except (ValueError, IndexError, AttributeError, TypeError):
            raise WheelInputError('foreign_sender') from None
        if len(data) > MAX_BYTES:
            raise WheelInputError('oversize_datagram')
        try:
            packet = json.loads(data, object_pairs_hook=_object, parse_constant=_nonfinite)
        except WheelInputError:
            raise
        except (ValueError, UnicodeError, RecursionError):
            raise WheelInputError('invalid_json') from None
        if not isinstance(packet, dict) or set(packet) != FIELDS:
            raise WheelInputError('invalid_fields')
        if packet['schema'] != SCHEMA:
            raise WheelInputError('schema_mismatch')
        if packet['source_id'] != self.options['source_id']:
            raise WheelInputError('source_mismatch')
        if packet['device_key'] != self.options['device_key']:
            raise WheelInputError('device_mismatch')
        stream = packet['stream_id']
        if not isinstance(stream, str) or not stream.strip() or len(stream) > 128:
            raise WheelInputError('invalid_stream_id')
        for field in ('seq', 'pc_query_ns', 'pc_send_ns'):
            minimum = 0 if field == 'seq' else 1
            if type(packet[field]) is not int or not minimum <= packet[field] <= 2**63-1:
                raise WheelInputError('invalid_' + field)
        if packet['pc_send_ns'] < packet['pc_query_ns']:
            raise WheelInputError('send_before_query')
        if type(packet['valid']) is not bool:
            raise WheelInputError('invalid_valid_flag')
        if type(packet['axis_min']) is not int or type(packet['axis_max']) is not int or (packet['axis_min'], packet['axis_max']) != (0, 65535):
            raise WheelInputError('axis_range_mismatch')
        reason = packet['reason']
        if not isinstance(reason, str) or len(reason) > 256:
            raise WheelInputError('invalid_device_reason')
        if packet['valid']:
            if type(packet['axis_raw']) is not int or not 0 <= packet['axis_raw'] <= 65535:
                raise WheelInputError('invalid_axis_raw')
            if reason:
                raise WheelInputError('valid_packet_has_reason')
        elif packet['axis_raw'] is not None or not reason.strip():
            raise WheelInputError('invalid_packet_requires_null_axis_and_reason')
        if stream == self._stream:
            if packet['seq'] <= self._seq:
                raise WheelInputError('sequence_not_increasing')
            if packet['pc_query_ns'] <= self._query or packet['pc_send_ns'] <= self._send:
                raise WheelInputError('pc_clock_not_increasing')
        else:
            if stream in self._seen_streams:
                raise WheelInputError('retired_stream')
            if len(self._seen_streams) >= MAX_STREAMS:
                raise WheelInputError('stream_capacity_exhausted')
        return packet

    def _observe(self, packet, now):
        self.last = None  # A logging failure must not leave the prior value usable.
        stream = packet['stream_id']
        offset = now - packet['pc_send_ns']
        self._offset_min = offset if stream != self._stream else min(self._offset_min, offset)
        self._seen_streams.add(stream)
        self._stream, self._seq = stream, packet['seq']
        self._query, self._send = packet['pc_query_ns'], packet['pc_send_ns']
        pc_age_ns = packet['pc_send_ns'] - packet['pc_query_ns']
        queue_age_ns = offset - self._offset_min
        age_ns = pc_age_ns + queue_age_ns
        norm = (2.*packet['axis_raw']/65535.-1.) if packet['valid'] else None
        physical = norm*self.options['physical_span_deg']/2 if norm is not None else None
        calibration = self.options['calibration']
        angle = (math.radians(norm*calibration['game_gain_deg']+calibration['game_offset_deg'])
                 if norm is not None and calibration['confirmed'] else None)
        reason = ('device_invalid' if not packet['valid'] else
                  'pc_query_stale' if pc_age_ns >= self.options['max_pc_age_ms']*1e6 else
                  'relative_delay_stale' if age_ns >= self.options['fresh_ms']*1e6 else
                  'calibration_unconfirmed' if not calibration['confirmed'] else None)
        # Record valid wire data even when unavailable to control. Journal errors
        # deliberately propagate to the app's existing logging safety handling.
        self.journal.record('wheel_sample', host_monotonic_ns=now, packet=dict(packet),
                            pc_age_ms=pc_age_ns/1e6, relative_queue_age_ms=queue_age_ns/1e6,
                            offset_min_ns=self._offset_min, norm=norm,
                            physical_angle_deg=physical, angle_rad=angle,
                            control_available=reason is None, unavailable_reason=reason,
                            timestamp_kind='pc_host_query_not_usb_capture',
                            absolute_network_latency_known=False)
        self.validated += 1
        if reason is not None:
            self._reject(reason, now=now)
            return
        self.last = dict(norm=norm, physical_angle_deg=physical, angle_rad=angle,
                         pc_query_ns=packet['pc_query_ns'], pc_send_ns=packet['pc_send_ns'],
                         sequence=packet['seq'], seq=packet['seq'], source_id=packet['source_id'],
                         stream_id=stream, device_key=packet['device_key'],
                         host_received_ns=now, host_query_estimate_ns=now-age_ns,
                         pc_age_s=pc_age_ns/1e9, relative_queue_age_s=queue_age_ns/1e9)
        self.accepted += 1
        self.last_reason = 'ok'

    def poll(self):
        if self.closed:
            return
        for index in range(MAX_DATAGRAMS+1):
            try:
                data, peer = self.s.recvfrom(MAX_BYTES+1)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                self._reject('socket_error')
                break
            self.received += 1
            try:
                now = self._now()
            except WheelInputError as exc:
                self._reject(str(exc), byte_count=len(data))
                break
            if index == MAX_DATAGRAMS:
                self._reject('datagram_backlog', now=now, byte_count=len(data))
                break
            try:
                packet = self._parse(data, peer)
            except WheelInputError as exc:
                self._reject(str(exc), now=now, byte_count=len(data))
                continue
            self._observe(packet, now)

    def latest(self):
        self.poll()
        if self.closed or self.last is None:
            return None, 1e9
        try:
            now = self._now()
        except WheelInputError as exc:
            self._reject(str(exc))
            return None, 1e9
        age = (now-self.last['host_query_estimate_ns'])/1e9
        if not 0 <= age < self.options['fresh_ms']/1000:
            self._reject('stale_sample', now=now)
            return None, 1e9
        return dict(self.last), age

    def summary(self):
        return dict(source='udp', enabled=True, closed=self.closed,
                    source_id=self.options['source_id'], device_key=self.options['device_key'],
                    listen=list(self.options['listen']), allow_from=sorted(self.options['allow_from']),
                    calibration=dict(self.options['calibration']),
                    physical_span_deg=self.options['physical_span_deg'],
                    fresh_ms=self.options['fresh_ms'], max_pc_age_ms=self.options['max_pc_age_ms'],
                    received=self.received, validated=self.validated, accepted=self.accepted,
                    reasons=dict(self.reasons), last_reason=self.last_reason,
                    stream_id=self._stream, sequence=self._seq, seen_streams=len(self._seen_streams),
                    offset_min_ns=self._offset_min, timestamp_kind='pc_host_query_not_usb_capture',
                    absolute_network_latency_known=False)

    def close(self):
        if not self.closed:
            self.closed = True
            self.last = None
            self.s.close()


def make_wheel(cfg, clock_ns, journal, allow_udp=True, transport=None):
    options = validate_config(cfg)
    if options['source'] == 'none':
        return NoWheel()
    if not allow_udp and transport is None:
        raise ValueError('SIL wheel input requires an injected transport')
    try:
        return UdpWheel(options, clock_ns, journal, transport=transport)
    except WheelBindError as exc:
        journal.record('wheel_unavailable', reason='bind_error', detail=str(exc))
        return NoWheel('bind_error')
