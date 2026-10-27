"""Pressure packet parser and uint32 tracker, copied unchanged from seat12_core.py v9."""

def parse_packet(line):
    """Return (packet_seq, MCU tick, channel, [(raw or None, status), ...])."""
    fields = line.split(',')
    if len(fields) != 9:
        raise ValueError('Expected nine CSV fields')
    seq, tick, channel = map(int, fields[:3])
    if not (0 <= seq <= 0xffffffff and 0 <= tick <= 0xffffffff and 0 <= channel <= 15):
        raise ValueError('Packet header outside range')
    readings = []
    for index in range(3):
        raw_text, status_text = fields[3 + 2 * index:5 + 2 * index]
        raw = int(raw_text) if raw_text else None
        status = int(status_text)
        if not 0 <= status <= 5:
            raise ValueError('ADC status outside range')
        if status == 0 and (raw is None or not 0 <= raw <= 4095):
            raise ValueError('Valid ADC reading missing or outside 12-bit range')
        if status != 0 and raw is not None:
            raise ValueError('Failed ADC conversion must have an empty raw field')
        readings.append((raw, status))
    return seq, tick, channel, readings


class SequenceTracker:
    """Unwrap uint32 sequence/timestamp, reject resets, count missing packets."""
    def __init__(self):
        self.last_seq = self.last_tick = self.first_extended = None
        self.extended = 0
        self.missing = self.duplicates = 0

    def accept(self, seq, tick):
        if self.last_seq is not None:
            delta = (seq - self.last_seq) & 0xffffffff
            if delta == 0:
                self.duplicates += 1
                return None
            if delta >= 0x80000000:
                raise RuntimeError('MCU 데이터 순번이 되돌아갔습니다.')
            dt = (tick - self.last_tick) & 0xffffffff
            if dt >= 0x80000000:
                raise RuntimeError('MCU 시간이 되돌아갔습니다.')
            self.extended += dt
            self.missing += delta - 1
        else:
            self.extended = self.first_extended = tick
        self.last_seq, self.last_tick = seq, tick
        return self.extended, (self.extended - self.first_extended) / 1000.0
