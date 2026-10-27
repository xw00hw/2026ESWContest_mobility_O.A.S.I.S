"""Pi pressure journal using the FB V7 schema and bounded asynchronous writer.

Imported only after fbstack.load() selects the FB source tree. No disk I/O runs
on the control thread after setup; a dropped acquisition record is a latched
logging failure even if the surviving records can still be written.
"""
from experiment_a.session_logging import SessionLogger


class PressureLogger(SessionLogger):
    def __init__(self, output, **kwargs):
        super().__init__(output, pressure_enabled=True, **kwargs)

    def pressure(self, records):
        super().pressure(records)
        if any(r.get('reason') == 'record_queue_overflow' for r in records):
            self._latch('pressure_record_queue_overflow: acquisition records lost; session incomplete')
            self.check()
