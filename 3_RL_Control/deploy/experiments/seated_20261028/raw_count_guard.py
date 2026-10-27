"""Absolute per-pad 2500-count release guard; no force conversion or baseline.

Each raw sample is observed before FbLink can replace it with a newer sweep.
No image, UART or controller is opened by this module. A latch lasts for the
whole trial. Zero ADC counts are valid no-contact observations.
"""
import math
import time

CHANNEL_SIDE = {'A1_C0': 1, 'A1_C1': 1, 'A1_C2': 0, 'A1_C3': 0}


class RawCountGuard:
    threshold = 2500
    stale_s = .3

    def __init__(self, provider, *, clock=time.monotonic, journal=None):
        self.provider, self.clock, self.journal = provider, clock, journal
        self.active = False
        self.latched = [False, False]
        self.reasons = [None, None]
        self.events = []
        self.observed_samples = 0

    @staticmethod
    def valid_count(value):
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value <= 4095

    def _latch(self, side, reason, *, channel=None, raw=None, acquired_ns=None):
        if not self.active or self.latched[side]:
            return
        self.latched[side] = True
        self.reasons[side] = reason
        event = dict(side=('left','right')[side], reason=reason, channel=channel,
                     raw_counts=raw, threshold_counts=self.threshold, acquired_ns=acquired_ns)
        self.events.append(event)
        if self.journal is not None:
            self.journal.record('raw_pressure_release_latched', host_monotonic_ns=int(self.clock()*1e9), **event)

    def _invalid(self, reason):
        for side in (0,1): self._latch(side, reason)

    def observe_records(self, records):
        """Called for every journal batch, never just the final retained sweep."""
        for record in records:
            if record.get('type') != 'sample':
                continue
            channel = record.get('channel')
            if type(channel) is not int or not 0 <= channel <= 3:
                continue
            self.observed_samples += 1
            raw = record.get('raw1')
            if not record.get('valid') or record.get('status1') != 0 or not self.valid_count(raw):
                self._invalid('invalid_side_sample')
                continue
            if raw >= self.threshold:
                name = 'A1_C'+str(channel)
                self._latch(CHANNEL_SIDE[name], 'raw_threshold', channel=name, raw=raw,
                            acquired_ns=record.get('host_monotonic_ns'))

    def snapshot(self):
        pressure, stamp = self.provider()
        now = self.clock()
        if not isinstance(stamp,(int,float)) or isinstance(stamp,bool) or not math.isfinite(stamp) or stamp > now or now-stamp > self.stale_s:
            self._invalid('pressure_stale_or_invalid_timestamp')
            return False, 'pressure_stale_or_invalid_timestamp'
        if not isinstance(pressure,dict) or any(not self.valid_count(pressure.get(k)) for k in CHANNEL_SIDE):
            self._invalid('pressure_missing_or_invalid')
            return False, 'pressure_missing_or_invalid'
        high = [k for k in CHANNEL_SIDE if pressure[k] >= self.threshold]
        for name in high:
            self._latch(CHANNEL_SIDE[name], 'raw_threshold', channel=name, raw=pressure[name], acquired_ns=int(stamp*1e9))
        return not high, ('raw_threshold:'+','.join(high) if high else None)

    def prearm_check(self):
        if self.active or any(self.latched):
            raise RuntimeError('Raw pressure guard cannot authorize a second ARM')
        okay, why = self.snapshot()
        if not okay:
            raise RuntimeError('No ARM: '+str(why))

    def activate(self):
        self.prearm_check()
        self.active = True

    def gate(self, pose):
        self.snapshot()
        value = list(pose)
        for side in (0,1):
            if self.latched[side]: value[2*side:2*side+2] = [0.,0.]
        return tuple(value)

    def permits_mdeg(self, ly, ry):
        self.snapshot()
        return (not self.latched[0] or ly == 0) and (not self.latched[1] or ry == 0)

    def summary(self):
        return dict(threshold_counts=self.threshold, threshold_comparison='>=',
                    channels=CHANNEL_SIDE, absolute_raw_before_baseline=True,
                    each_valid_sample_observed=True, observed_samples=self.observed_samples,
                    active=self.active, latched=self.latched, reasons=self.reasons, events=self.events,
                    stale_s=self.stale_s, force_units_calibrated=False)


def attach_pressure_observer(journal, guard):
    original = journal.pressure
    def pressure(records):
        guard.observe_records(records)
        original(records)
    journal.pressure = pressure


def attach_service_gate(link, guard, limiter):
    """Re-evaluate after polling and before UART service, even between RL ticks."""
    service, send = link.service, link.send_targets
    def gated_service():
        if guard.active:
            requested = tuple(v/1000 for v in link.target)
            gated = limiter.degrees(requested)
            if tuple(round(v*1000) for v in gated) != link.target:
                send(*gated)
        return service()
    link.service = gated_service
