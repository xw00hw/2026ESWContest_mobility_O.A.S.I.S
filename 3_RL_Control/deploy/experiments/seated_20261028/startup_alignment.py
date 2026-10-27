"""Bounded receive alignment BEFORE the first protocol request; no UART write.

Never applied to an active session or used to ignore a runtime protocol error.
Discard only pre-handshake bytes until a complete newline and quiet interval,
or fail closed within half a second. The regular parser remains unchanged.
"""
import math
import time


class StartupAlignmentError(RuntimeError):
    def __init__(self, reason, report):
        super().__init__('Startup UART alignment failed: '+reason)
        self.report = dict(report, reason=reason, success=False)


def align_startup_input(raw, *, clock=time.monotonic, sleep=time.sleep,
                        quiet_s=.010, timeout_s=.5, trace=None):
    if getattr(raw,'timeout',None) != 0 or getattr(raw,'write_timeout',None) != 0:
        raise ValueError('Startup alignment requires nonblocking transport')
    if not (math.isfinite(quiet_s) and math.isfinite(timeout_s) and 0 < quiet_s < timeout_s <= .5):
        raise ValueError('Invalid bounded startup alignment timing')
    start=clock(); last_rx=start; deadline=start+timeout_s
    report=dict(success=False, discarded_bytes=0, discarded_newlines=0,
                quiet_s=quiet_s, timeout_s=timeout_s, elapsed_s=0., ended_at_line_boundary=True,
                hardware_commands_sent=0, input_reset_calls=0)
    def fail(reason):
        report['elapsed_s']=max(0.,clock()-start)
        raise StartupAlignmentError(reason,report)
    raw.reset_input_buffer(); report['input_reset_calls']=1
    boundary=True; last_clock=start
    while True:
        now=clock()
        if not math.isfinite(now) or now < last_clock: fail('clock_invalid_or_regressed')
        last_clock=now
        if now >= deadline: fail('timeout_before_quiet_line_boundary')
        waiting=raw.in_waiting
        if type(waiting) is not int or waiting < 0 or waiting > 32768: fail('invalid_or_excessive_backlog')
        if waiting:
            data=raw.read(min(waiting,4096))
            if type(data) is not bytes or len(data)>min(waiting,4096): fail('invalid_read')
            if data:
                if trace is not None: trace(data)
                report['discarded_bytes']+=len(data)
                if report['discarded_bytes']>32768: fail('discard_budget_exceeded')
                report['discarded_newlines']+=data.count(b'\n')
                boundary=data.endswith(b'\n')
                report['ended_at_line_boundary']=boundary
                last_rx=clock()
        now=clock()
        if now >= deadline: fail('timeout_before_quiet_line_boundary')
        if boundary and now-last_rx >= quiet_s:
            report.update(success=True,reason='quiet_after_line_boundary' if report['discarded_bytes'] else 'quiet_without_input',elapsed_s=now-start)
            return report
        sleep(min(.001,max(0.,deadline-clock())))
