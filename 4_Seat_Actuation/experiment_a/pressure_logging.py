"""Bounded, observation-only FSR records; never supplies controller inputs.

The v10 scanner emits C0..C3 rows, each containing sequential A0/A1/A2
12-bit ADC readings, nominally 100 sweeps/s. Voltages use nominal 3.3 V;
they are not calibrated pressure/force. MCU tick is acquisition time in ms;
host_monotonic_ns is receipt time and includes UART/host delay. A sweep
retains each row's timestamps rather than pretending all 12 are simultaneous.
"""
from __future__ import annotations

from collections import deque

from fallback.serial_link import UINT32_MAX, _now, _uint


class PressureClockReset(ValueError):
    """A backwards MCU acquisition clock is evidence of a board reset."""


class PressureRecorder:
    def __init__(self, queue_limit=2048):
        if type(queue_limit) is not int or not 4 <= queue_limit <= 100000:
            raise ValueError("pressure_queue_limit must be an integer in 4..100000")
        self.queue_limit = queue_limit
        self.records = deque()
        self.counters = dict(samples=0, malformed=0, invalid_samples=0,
                             complete_sweeps=0, invalid_sweeps=0, sequence_gaps=0,
                             missing_samples=0, queue_dropped=0, stats_rows=0)
        self.stats = {}
        self._last = self._anchor = None
        self._elapsed_ms = 0
        self._parts = []
        self._dropped = 0
        self._drop_first_ns = self._drop_last_ns = None
        self.last_sample_ns = self.last_valid_sweep_ns = None
        self.last_reason = "no_samples"

    def _emit(self, record):
        if len(self.records) >= self.queue_limit:
            lost = self.records.popleft()
            self.counters["queue_dropped"] += 1
            self._dropped += 1
            self._drop_last_ns = lost["host_monotonic_ns"]
            if self._drop_first_ns is None:
                self._drop_first_ns = self._drop_last_ns
        self.records.append(record)

    def event(self, now_ns, reason, *, raw_line=None, **details):
        self._emit(dict(type="event", host_monotonic_ns=_now(now_ns),
                        reason=reason, raw_line=raw_line, **details))

    def _incomplete(self, now_ns, reason):
        if self._parts:
            self.event(now_ns, "incomplete_sweep", cause=reason,
                       retained_sequences=[part["seq"] for part in self._parts])
            self._parts.clear()

    def sample(self, line, now_ns):
        now_ns = _now(now_ns)
        fields = line.split(",")
        self.counters["samples"] += 1
        self.last_sample_ns = now_ns
        try:
            if len(fields) != 9:
                raise ValueError("Expected nine-field FSR sample")
            seq, tick, channel = _uint(fields[0]), _uint(fields[1]), _uint(fields[2], 3)
            raw, status = [], []
            for bank in range(3):
                text, status_text = fields[3+2*bank:5+2*bank]
                status.append(_uint(status_text, 5))
                # An ADC error is transmitted with an empty raw field.
                raw.append(None if not text and status[-1] else _uint(text, 4095))
        except ValueError as exc:
            self.counters["malformed"] += 1
            self.last_reason = "malformed_sample"
            self._incomplete(now_ns, self.last_reason)
            self.event(now_ns, self.last_reason, raw_line=line, detail=str(exc))
            return

        issues = []
        lost = 0
        fatal_reset = False
        if self._last is not None:
            previous_seq, previous_tick, previous_host = self._last
            sequence_delta = (seq-previous_seq) & UINT32_MAX
            tick_delta = (tick-previous_tick) & UINT32_MAX
            if sequence_delta != 1:
                issues.append("sequence_gap_or_duplicate")
                self.counters["sequence_gaps"] += 1
                if 1 < sequence_delta <= 0x7fffffff:
                    lost = sequence_delta-1
                    self.counters["missing_samples"] += lost
            if tick_delta > 0x7fffffff:
                issues.append("mcu_clock_regression")
                fatal_reset = True
            elif tick_delta == 0 or tick_delta > 50:
                issues.append("mcu_tick_gap_or_duplicate")
            if now_ns-previous_host > 100_000_000:
                issues.append("host_receipt_gap")
            if not fatal_reset:
                self._elapsed_ms += tick_delta
        else:
            self._anchor = now_ns
        if self._anchor is not None and now_ns-self._anchor-self._elapsed_ms*1_000_000 > 100_000_000:
            issues.append("transport_delay")
        if any(status):
            issues.append("adc_status")
        if issues:
            self._incomplete(now_ns, ";".join(issues))
            self._anchor, self._elapsed_ms = now_ns, 0
        self._last = (seq, tick, now_ns)
        valid = not issues
        if not valid:
            self.counters["invalid_samples"] += 1
        self.last_reason = "ok" if valid else ";".join(issues)
        # Stable raw fields match the wire CSV and can be flattened directly.
        record = dict(type="sample", host_monotonic_ns=now_ns, raw_line=line,
                      seq=seq, t_ms=tick, channel=channel, valid=valid,
                      reason=self.last_reason, missing_samples_before=lost)
        for bank in range(3):
            record[f"raw{bank}"] = raw[bank]
            record[f"status{bank}"] = status[bank]
            record[f"voltage{bank}_v"] = (raw[bank]*3.3/4095.
                if raw[bank] is not None and status[bank] == 0 else None)
        self._emit(record)
        if fatal_reset:
            raise PressureClockReset("MCU pressure clock reset")

        if channel == 0:
            self._incomplete(now_ns, "new_channel_zero")
            self._parts = [record]
        elif len(self._parts) == channel:
            self._parts.append(record)
        else:
            self._incomplete(now_ns, "channel_order")
            self.event(now_ns, "incomplete_sweep", cause="channel_order", seq=seq, channel=channel)
        if len(self._parts) == 4:
            parts, self._parts = self._parts, []
            sweep_valid = all(part["valid"] for part in parts)
            values = {}
            for part in parts:
                for bank in range(3):
                    sensor = f"A{bank}_C{part['channel']}"
                    values[sensor] = dict(raw=part[f"raw{bank}"],
                        status=part[f"status{bank}"], voltage_v=part[f"voltage{bank}_v"],
                        host_monotonic_ns=part["host_monotonic_ns"], t_ms=part["t_ms"], seq=part["seq"])
            self.counters["complete_sweeps"] += 1
            if sweep_valid:
                self.last_valid_sweep_ns = parts[0]["host_monotonic_ns"]
            else:
                self.counters["invalid_sweeps"] += 1
            self._emit(dict(type="sweep", host_monotonic_ns=parts[0]["host_monotonic_ns"],
                completed_host_monotonic_ns=now_ns, first_seq=parts[0]["seq"],
                last_seq=seq, first_t_ms=parts[0]["t_ms"], last_t_ms=tick,
                valid=sweep_valid, reason="ok" if sweep_valid else "invalid_component",
                values=values))

    def statistics(self, line, now_ns):
        self.counters["stats_rows"] += 1
        try:
            values = {}
            for item in line.split(",")[1:]:
                key, separator, text = item.partition("=")
                if not separator or not key or key in values:
                    raise ValueError("Malformed FSR statistics")
                values[key] = _uint(text)
        except ValueError as exc:
            self.event(now_ns, "malformed_stats", raw_line=line, detail=str(exc))
            return {}
        changed = {key: value for key, value in values.items() if self.stats.get(key) != value}
        self.stats.update(values)
        self._emit(dict(type="stats", host_monotonic_ns=now_ns, raw_line=line,
                        values=values, changed=changed))
        return values

    def status(self, now_ns):
        now_ns = _now(now_ns)
        return dict(self.counters, queue_size=len(self.records), queue_limit=self.queue_limit,
                    last_sample_age_ns=None if self.last_sample_ns is None else now_ns-self.last_sample_ns,
                    last_valid_sweep_age_ns=None if self.last_valid_sweep_ns is None else now_ns-self.last_valid_sweep_ns,
                    last_reason=self.last_reason, firmware_stats=dict(self.stats))

    def drain(self):
        records = list(self.records)
        self.records.clear()
        if self._dropped:
            records.insert(0, dict(type="event", host_monotonic_ns=self._drop_last_ns,
                reason="record_queue_overflow", raw_line=None, dropped_records=self._dropped,
                first_dropped_host_monotonic_ns=self._drop_first_ns,
                last_dropped_host_monotonic_ns=self._drop_last_ns))
            self._dropped = 0
            self._drop_first_ns = self._drop_last_ns = None
        return records
