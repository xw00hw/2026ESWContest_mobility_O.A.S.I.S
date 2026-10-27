"""BeamNG vehicle input on the Pi, following BeamNG_Bolster_FB's collector path.

  PC : BeamNG.drive -> BeamNG Collector v0.3 (UDP 64256, raw archive) -> fallback.beamng_bridge (copy, 127.0.0.1:64257)
       -> 3_RL_Control/beamng/pi_relay.py -> Pi UDP 64257
  datagram = b'OASIS-RELAY1 <pc_age_ns> <relay_seq>\\n' + the FB bridge datagram, byte-identical
             (schema oasis.beamng.fallback.v1, checked with fallback.beamng.read_bridge_packet)

Clocks: the collector stamps host_monotonic_ns with the PC's perf_counter; the Pi cannot compare that with its own
clock. The relay measures the PC-side age (collector receipt -> relay send, same PC clock) and the Pi re-stamps
  host_monotonic_ns := Pi receipt - pc_age     (original kept as _pc_host_monotonic_ns)
so FB's VehicleValidator freshness / gap / order gates (200 ms) run on the Pi clock and cover collector -> Pi use,
except the network transit itself (wired LAN, ~1 ms; not measurable without clock sync).

VehicleInput maps a validated frame to the runtime's vehicle sample (+x fwd, +y LEFT):
  ay_mps2, ax_mps2   linear_acc_y/x_mps2 (gravity excluded, the collector's velocity derivative)
  speed_mps          speed_mps
  steer_rad          steer_sign * radians(steering_angle_deg)   (handwheel angle; + = left with steer_sign 1)
  yaw_rate_radps     not in the collector schema -> a_y / max(v, 5 m/s) (steady-state kinematic estimate)
  t                  sim_time_s (the predictor's wheel-rate clock)
`stopped` rows (speed < 1 m/s) are usable (bolsters open), every other invalid row is an input fault.
"""
import ipaddress, math, socket, time
from collections import Counter

HEADER = b'OASIS-RELAY1 '
MAX_DATAGRAM = 16384 + 64


def udp_endpoint(host, port):
    """Resolve IPv4/IPv6 UDP endpoints, retaining the OS scope ID for link-local IPv6 destinations.

    Resolve interface names (%eth0) to numeric zones (%12) before getaddrinfo for Windows portability.
    Brackets are optional here;
    the relay CLI uses [::1]:64257 to separate an IPv6 listener from its port.
    """
    host = str(host)
    if host.startswith('[') and host.endswith(']'): host = host[1:-1]
    if ':' in host and '%' in host:
        address, zone = host.rsplit('%', 1)
        if not zone.isdecimal(): host = f'{address}%{socket.if_nametoindex(zone)}'
    family, _, _, _, address = socket.getaddrinfo(host, int(port), socket.AF_UNSPEC,
                                                 socket.SOCK_DGRAM, socket.IPPROTO_UDP)[0]
    return family, address


def peer_ip(host):
    """Canonical peer identity; a zone selects a local interface, not a different remote IP address."""
    return str(ipaddress.ip_address(str(host).split('%', 1)[0]))


def relay_wrap(payload, pc_age_ns, seq):
    return HEADER + f'{int(pc_age_ns)} {int(seq)}\n'.encode('ascii') + payload


def relay_unwrap(data):
    if not data.startswith(HEADER) or b'\n' not in data[:64]:
        raise ValueError('not an OASIS-RELAY1 datagram')
    head, payload = data.split(b'\n', 1)
    age, seq = head[len(HEADER):].split(b' ')
    age, seq = int(age), int(seq)
    if not 0 <= age < 10_000_000_000 or seq < 0: raise ValueError('bad relay header')
    return age, seq, payload


class RelayReceiver:
    """Non-blocking UDP receiver for pi_relay.py datagrams. Peers outside `allow` are dropped and counted."""
    def __init__(self, fb, listen=('0.0.0.0', 64257), allow=None, clock_ns=time.monotonic_ns, max_pc_age_ms=50.0):
        self.fb, self.clock_ns, self.allow = fb, clock_ns, {peer_ip(host) for host in (allow or [])}
        self.max_pc_age_ns = int(max_pc_age_ms * 1e6)
        family, address = udp_endpoint(*listen)
        self.s = socket.socket(family, socket.SOCK_DGRAM)
        try:
            if family == socket.AF_INET6: self.s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 16)
            self.s.bind(address); self.s.setblocking(False)
        except Exception:
            self.s.close(); raise
        self.last_stamp = 0; self.foreign = 0; self.pc_age_ms = []; self.relay_seq = None; self.relay_gaps = 0

    def receive(self):
        try:
            data, peer = self.s.recvfrom(MAX_DATAGRAM)
        except (BlockingIOError, InterruptedError):
            return None
        now = self.clock_ns()
        if self.allow and peer_ip(peer[0]) not in self.allow:
            self.foreign += 1; return None
        age, seq, payload = relay_unwrap(data)
        if self.relay_seq is not None and seq != self.relay_seq + 1: self.relay_gaps += 1
        self.relay_seq = seq
        row = self.fb.beamng.read_bridge_packet(payload)
        self.pc_age_ms.append(age / 1e6); del self.pc_age_ms[:-2000]
        stamp = max(now - age, self.last_stamp + 1); self.last_stamp = stamp
        row['_pc_host_monotonic_ns'] = row['host_monotonic_ns']; row['host_monotonic_ns'] = stamp
        if age > self.max_pc_age_ns: row['derivative_reason'] = 'pc_forward_delay'    # fails FB's quality gate
        return row

    def close(self): self.s.close()


class SteerSignMonitor:
    """Steering sign vs measured a_y while cornering (v > 8 m/s, |a_y| > 1.5 m/s^2, |wheel| > 10 deg).
    verdict: None (not enough evidence yet) | 'ok' | 'mismatch' (agreement < 20 % over >= n_min rows)."""
    def __init__(self, n_min=100):
        self.n_min = n_min; self.n = self.agree = 0
    def update(self, ay, v, steer_deg_signed):
        if ay is None or v is None or steer_deg_signed is None: return
        if v > 8 and abs(ay) > 1.5 and abs(steer_deg_signed) > 10:
            self.n += 1; self.agree += int((steer_deg_signed > 0) == (ay > 0))
    @property
    def fraction(self): return self.agree / self.n if self.n else None
    @property
    def verdict(self):
        if self.n < self.n_min: return None
        return 'ok' if self.fraction >= 0.2 else 'mismatch'


class VehicleInput:
    """Drains the receiver every pump, validates EVERY row with FB's VehicleValidator (a later good row cannot hide
    an earlier bad one), keeps the latest frame and the first input issue. Vehicle-source interface for Controller.
    With FB V7 every parsed row also feeds FB's ImpactDetector (latched): a suspected impact is an input issue, so the
    session turns OFF and ends like any other input fault (the Pi has no record-only mode)."""
    def __init__(self, fb, receiver, clock_ns, fresh_ms=200.0, steer_sign=1, allow_synthetic=False,
                 require_steering=True, yaw_rate_min_v=5.0, wheel=None, wheel_priority=False, journal=None,
                 wheel_vehicle_id=None):
        self.fb, self.rx, self.clock_ns = fb, receiver, clock_ns
        self.validator = fb.vehicle.VehicleValidator(allow_synthetic=allow_synthetic, max_age_ms=fresh_ms,
                                                     max_host_gap_ms=fresh_ms, max_sim_dt_s=0.1)
        self.fresh_ns = int(fresh_ms * 1e6); self.sign = int(steer_sign); self.need_steer = require_steering
        self.vmin = yaw_rate_min_v
        self.frame = None; self.issue = None; self.n = 0; self.reasons = Counter(); self.sign_mon = SteerSignMonitor()
        self.impact = fb.impact.ImpactDetector(fb.settings.Settings()) if getattr(fb, 'impact', None) else None
        self.wheel, self.wheel_priority, self.journal = wheel, wheel_priority, journal
        self.wheel_cleanup_error = None
        self.wheel_vehicle_id = wheel_vehicle_id

    def reset_issue(self): self.issue = None

    def _usable(self, f):
        if not (f.valid or f.reason == 'stopped'): return f.reason
        if self.need_steer and f.steering_deg is None: return 'missing:steering_angle_deg'
        return None

    def poll(self, max_rows=64):
        if self.wheel is not None: self.wheel.poll()
        for _ in range(max_rows):
            try:
                row = self.rx.receive()
            except (ValueError, OSError) as e:
                self.issue = self.issue or f'malformed:{e}'; self.reasons['malformed'] += 1; continue
            if row is None: break
            try:
                f = self.validator.parse(row, self.clock_ns())
            except ValueError as e:
                self.issue = self.issue or f'invalid_row:{e}'; self.reasons['invalid_row'] += 1; continue
            self.frame = f; self.n += 1; self.reasons[f.reason] += 1
            if self.impact is not None:
                try:
                    hit = self.impact.observe(f)
                    if hit.triggered: self.issue = self.issue or f'impact:{hit.reason}'
                except ValueError as e:
                    self.issue = self.issue or f'impact_input:{e}'
            bad = self._usable(f)
            if bad: self.issue = self.issue or bad
            elif f.steering_deg is not None:
                self.sign_mon.update(f.ay_mps2, f.speed_mps, self.sign * f.steering_deg)
        return self.frame

    def good(self):
        f = self.frame
        return f is not None and self._usable(f) is None and 0 <= self.clock_ns() - f.received_monotonic_ns <= self.fresh_ns

    def sample(self):
        f = self.frame
        v = f.speed_mps if f.speed_mps is not None else 0.0
        ay = f.ay_mps2 if f.ay_mps2 is not None else 0.0
        sw = self.sign * math.radians(f.steering_deg or 0.0)
        r = ay / max(v, self.vmin) if v > 0.5 else 0.0
        sample = dict(t=f.sim_time_s, ay_mps2=ay, ax_mps2=f.ax_mps2 if f.ax_mps2 is not None else 0.0, speed_mps=v,
                      steer_rad=sw, steer_input_rad=sw, yaw_rate_radps=r, road_valid=0.0)
        if self.wheel is not None and self.wheel_priority:
            direct, age = self.wheel.latest()
            vehicle_match = self.wheel_vehicle_id is None or str(f.vehicle_id) == self.wheel_vehicle_id
            if not vehicle_match: direct = None
            sample['_moza_valid'] = direct is not None
            if direct is not None:
                # Calibration is against raw Collector game steering; apply the
                # same game-to-vehicle sign used by measured BeamNG steering.
                sample.update(steer_input_rad=self.sign * direct['angle_rad'], _moza_query_ns=direct['pc_query_ns'],
                              _moza_stream_id=direct['stream_id'], _moza_age_s=age)
            if self.journal is not None:
                self.journal.record('wheel_selection', host_monotonic_ns=self.clock_ns(),
                    selected='moza_wheel' if direct is not None else 'beamng_predictor',
                    beamng_steer_rad=sw, predictor_steer_rad=sample['steer_input_rad'],
                    wheel_age_s=age if direct is not None else None,
                    wheel_query_ns=direct['pc_query_ns'] if direct is not None else None,
                    wheel_seq=direct['seq'] if direct is not None else None,
                    vehicle_matches_wheel_calibration=vehicle_match)
        return sample

    def latest(self):
        """(sample, age_s) for controller.Controller; (None, inf) when the frame is not usable."""
        if not self.good(): return None, 1e9
        return self.sample(), (self.clock_ns() - self.frame.received_monotonic_ns) / 1e9

    def close(self):
        try: self.rx.close()
        finally:
            if self.wheel is not None:
                try: self.wheel.close()
                except Exception as exc: self.wheel_cleanup_error = f'{type(exc).__name__}: {exc}'
