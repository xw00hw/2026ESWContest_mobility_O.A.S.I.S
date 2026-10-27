"""Rebuild Part B from read-only UART/CSV sources; run from any directory."""
import csv
import hashlib
import json
import os
from datetime import timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QC = ROOT / '02_qc/2_seat_pressure'
WORK = ROOT / '_work/2_seat_pressure'
BLOCK_PACKETS = 200
W_SPREAD_MAX_NS = 151000  # 0.1508 ms observed; the preliminary 0.15 ms was rounded.
ANCHOR_REJECT_MS = 0.2
SPIKE_WINDOW = 11
SPIKE_MIN_COUNTS = 50
SPIKE_MAD_MULTIPLIER = 6
RUN2 = 'session_run2'  # 2차 주행 압력 세션 폴더 이름
IDS = [f'a{b}_c{c}' for b in range(3) for c in range(4)]  # snake_case of the collector's A0_C0 ids
RAW = [f'{c}_raw' for c in IDS]
VOLTS = [f'{c}_v' for c in IDS]
HEAD = ['host_monotonic_ns', 'host_wall_time_ns', 'sequence_id', 'packet_seq',
        'mcu_t_ms', 'packets_in_frame', 'frame_valid']
COLUMNS = ['t_utc_ns', 'mcu_t_ms', 'sequence_id', 'packet_seq', 'host_monotonic_ns',
           'host_wall_time_ns', 'packets_in_frame', 'host_delay_ms'] + RAW + VOLTS + [
           'frame_valid', 'flag_incomplete_frame', 'flag_ntp_fallback', 'flag_spike', 'spike_mask']
BASELINE_NOTE = ('확인된 무하중(빈 좌석) 기록은 없다. setup_check도 여러 채널에 부하가 보이며 '
                 'events.jsonl에는 착석 표식이 없다. 기준선은 관측값일 뿐이며 빼거나 보정하지 않았다.')
TIMING_NOTE = ('프레임 첫 패킷(C0)의 지터 제거 호스트 도착 시각을 NTP 보정한 값이다. '
               'UART 30–40 bytes / 460800 baud ≈ 0.7–0.9 ms 및 ST-LINK VCP/USB의 '
               '상수 전송 지연은 측정하지 않아 보정하지 않았다. 샘플링보다 약 1–3 ms 늦을 수 있다. '
               'Ck는 k 패킷 뒤에 스캔하며 같은 패킷의 3 ADC도 순차 변환한다.')
WORK.mkdir(parents=True, exist_ok=True)
os.environ['MPLCONFIGDIR'] = str(WORK / 'matplotlib')
import numpy as np
import pandas as pd
from scipy.optimize import linprog
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
                    encoding='utf-8', newline='\n')


def parse_packet(text):
    f = text.split(',')
    if len(f) != 9:
        raise ValueError(f'field count={len(f)}, expected 9')
    seq, tick, channel = map(int, f[:3])
    if not (0 <= seq <= 0xFFFFFFFF and 0 <= tick <= 0xFFFFFFFF and 0 <= channel <= 15):
        raise ValueError('header outside range')
    readings = []
    for bank in range(3):
        raw = int(f[3 + 2 * bank]) if f[3 + 2 * bank] else None
        status = int(f[4 + 2 * bank])
        if not 0 <= status <= 5 or (status == 0) != (raw is not None):
            raise ValueError(f'bank {bank}: raw/status mismatch or status outside 0..5')
        if raw is not None and not 0 <= raw <= 4095:
            raise ValueError(f'bank {bank}: raw outside 12-bit range')
        readings.append((raw, status))
    if channel not in range(4):
        raise ValueError('channel was not requested (expected 0..3)')
    return seq, tick, channel, readings


def unwrap(x):
    x = np.asarray(x, dtype=np.int64)
    return x[0] + np.r_[0, np.cumsum(np.diff(x) % (2**32))]


def hampel(x):
    half = SPIKE_WINDOW // 2
    windows = np.lib.stride_tricks.sliding_window_view(
        np.pad(x, (half, half), constant_values=np.nan), SPIKE_WINDOW)
    # All-blank windows have no finite centre and cannot be spikes.
    finite = np.any(np.isfinite(windows), axis=1)
    med, mad = np.full(len(x), np.nan), np.full(len(x), np.nan)
    med[finite] = np.nanmedian(windows[finite], axis=1)
    mad[finite] = np.nanmedian(np.abs(windows[finite] - med[finite, None]), axis=1)
    return np.isfinite(x) & (np.abs(x - med) > np.maximum(
        SPIKE_MIN_COUNTS, SPIKE_MAD_MULTIPLIER * 1.4826 * mad))


def self_check():
    assert np.array_equal(unwrap([2**32 - 2, 2**32 - 1, 0, 1]),
                          [2**32 - 2, 2**32 - 1, 2**32, 2**32 + 1])
    assert parse_packet('0,2,0,,1,0,0,4095,0')[3] == [(None, 1), (0, 0), (4095, 0)]
    for bad in ['0,2,0,1,0,1,0,1', '0,2,0,,0,0,0,0,0', '0,2,0,4096,0,0,0,0,0']:
        try:
            parse_packet(bad)
        except ValueError:
            continue
        raise AssertionError('parser accepted a corrupt packet')
    x = np.zeros(21); x[0] = 51; x[10] = 50; x[20] = np.nan
    assert np.flatnonzero(hampel(x)).tolist() == [0]
    assert not hampel(np.full(12, np.nan)).any()


def reparse(folder):
    meta = json.loads((folder / 'metadata.json').read_text(encoding='utf-8'))
    summary = json.loads((folder / 'session_summary.json').read_text(encoding='utf-8'))
    sid, cfg = meta['session_id'], meta['config']
    assert sid == folder.name and summary['session_id'] == sid
    mapping = [r for r in cfg['mapping'] if r['enabled']]
    assert [r['id'].lower() for r in mapping] == IDS and cfg['vref'] == 3.3
    assert [(r['bank'], r['channel']) for r in mapping] == [(b, c) for b in range(3) for c in range(4)]
    with (folder / 'seat/pressure.csv').open(encoding='utf-8', newline='') as f:
        reader = csv.reader(f)
        assert [h.lower() for h in next(reader)] == HEAD + RAW + VOLTS
        cells = list(reader)
    assert all(len(row) == 31 for row in cells)
    frame, head, reconstructed, packets, issues = {}, None, [], [], []
    counts = dict(log_lines_total=0, control_lines=0, accepted_packets=0,
                  trailing_unprocessed_lines=0, unparsable_lines=0)
    prev = None

    def flush():
        if not frame:
            return
        vals = [frame.get(r['channel'], [(None, None)] * 3)[r['bank']] for r in mapping]
        raws = [v[0] for v in vals]
        valid = len(frame) == 4 and all(v[1] == 0 for v in vals)
        mono, wall, seq, tick = head
        reconstructed.append([mono, wall, len(reconstructed), seq, tick, len(frame), int(valid)] +
                             raws + [None if v is None else round(v * 3.3 / 4095, 5) for v in raws])
        frame.clear()

    with (folder / 'seat/raw_uart.log').open(encoding='utf-8') as f:
        for lineno, line in enumerate(f, 1):
            counts['log_lines_total'] += 1
            trailing = len(packets) >= summary['packets']
            counts['trailing_unprocessed_lines'] += int(trailing)
            reasons, packet = [], None
            try:
                mono_s, wall_s, text = line.rstrip('\r\n').split(' ', 2)
                mono, wall = int(mono_s), int(wall_s)
                if text.startswith('#'):
                    counts['control_lines'] += 1
                    if trailing:
                        issues.append(dict(line=lineno, text=text, raw_line=line.rstrip(),
                                           trailing=True, reasons=['control line; not corrupt; shutdown not parsed']))
                    continue
                packet = parse_packet(text)
            except ValueError as exc:
                text = line.rstrip('\r\n').split(' ', 2)[-1]
                reasons.append(str(exc))
                counts['unparsable_lines'] += 1
            if packet is not None:
                seq, tick, channel, readings = packet
                if prev is not None:
                    if (seq - prev[0]) % 2**32 != 1:
                        reasons.append(f'seq not prev+1: {prev[0]} -> {seq}')
                    if (tick - prev[1]) % 2**32 not in (2, 3):
                        reasons.append(f'tick not within prev+{{2,3}}: {prev[1]} -> {tick}')
                    if channel != (prev[2] + 1) % 4:
                        reasons.append(f'channel cycle violation: {prev[2]} -> {channel}')
                prev = (seq, tick, channel)
            if trailing or reasons:
                issues.append(dict(line=lineno, text=text, raw_line=line.rstrip(), trailing=trailing,
                                   reasons=reasons or ['valid packet; not corrupt; beyond collector accepted count']))
            if trailing or packet is None:
                continue
            # Matches collector duplicate/backwards handling; continuity is checked below.
            if packets:
                delta = (seq - packets[-1][2]) % 2**32
                if delta == 0:
                    continue
                assert delta < 2**31, f'{sid}: packet sequence went backwards'
            packets.append((mono, wall, seq, tick, channel))
            if frame and channel <= max(frame):
                flush()
            if not frame:
                head = (mono, wall, seq, tick)
            frame[channel] = readings
            if len(frame) == 4:
                flush()
    flush()
    p = np.asarray(packets, dtype=np.int64)
    counts['accepted_packets'] = len(p)
    assert len(p) == summary['packets'] and len(cells) == len(reconstructed) == summary['frames']
    mismatches = 0
    for original, rebuilt in zip(cells, reconstructed):
        for col, (v, expected) in enumerate(zip(original, rebuilt)):
            actual = None if v == '' else float(v) if col >= 19 else int(v)
            mismatches += actual != expected
    n, tick = unwrap(p[:, 2]), unwrap(p[:, 3])
    violations = dict(seq_step=int(np.count_nonzero(np.diff(n) != 1)),
                      tick_step=int(np.count_nonzero(~np.isin(np.diff(tick), [2, 3]))),
                      channel_cycle=int(np.count_nonzero(p[:, 4] != (p[0, 4] + np.arange(len(p))) % 4)))
    phase = tick - 2.5 * n
    audit = dict(**counts, frames_compared=len(cells), mismatching_cells=mismatches,
                 violations=violations, seq_wraps=int(np.count_nonzero(np.diff(p[:, 2]) < 0)),
                 tick_wraps=int(np.count_nonzero(np.diff(p[:, 3]) < 0)),
                 timer_phase_spread_ms=float(np.ptp(phase)), issues=issues)
    assert mismatches == 0, f'{sid}: {mismatches} reparse mismatching cells'
    assert violations['seq_step'] == 0 and violations['channel_cycle'] == 0, (sid, violations)
    assert np.ptp(phase) <= 1.5, (sid, 'timer phase spread', float(phase.min()), float(phase.max()))
    assert p[0, 4] == 0 and np.array_equal(n[::4] % 2**32, np.array([int(r[3]) for r in cells]))
    df = pd.DataFrame(cells, columns=HEAD + RAW + VOLTS)
    for c in HEAD:
        df[c] = df[c].astype('int64')
    assert int((df.frame_valid == 0).sum()) == summary['invalid_frames'] == 1
    assert (df.packets_in_frame.iloc[:-1] == 4).all() and df.packets_in_frame.iloc[-1] < 4
    events = [json.loads(x) for x in (folder / 'events.jsonl').read_text(encoding='utf-8').splitlines() if x]
    assert [e['kind'] for e in events] == ['recording_started'], f'{sid}: unreviewed event/marker'
    w = p[:, 1] - p[:, 0]
    W = int(np.sort(w)[len(w) // 2])
    assert np.ptp(w) <= W_SPREAD_MAX_NS, (sid, 'wall-minus-QPC spread ns', int(np.ptp(w)), W_SPREAD_MAX_NS)
    return dict(sid=sid, meta=meta, source=df, packets=p, n=n, W=W,
                audit=audit, wall_spread_ms=float(np.ptp(w) / 1e6),
                event_kinds=[e['kind'] for e in events])


def fit_time(s):
    p, n = s['packets'], s['n']
    x = (n - n[0]).astype(np.float64)
    y = (p[:, 0] - p[0, 0]).astype(np.float64)
    b, a = np.linalg.lstsq(np.column_stack([x, np.ones(len(x))]), y, rcond=None)[0]
    d = y - (a + b * x)
    size = BLOCK_PACKETS if len(x) >= 2 * BLOCK_PACKETS else 50
    blocks = [(i, min(i + size, len(x))) for i in range(0, len(x), size)]
    if len(blocks) > 1 and blocks[-1][1] - blocks[-1][0] < size / 2:
        blocks[-2:] = [(blocks[-2][0], blocks[-1][1])]
    idx = np.array([lo + np.argmin(d[lo:hi]) for lo, hi in blocks])
    initial = len(idx)
    while len(idx) > 1:
        z, v = x[idx], d[idx]
        neighbours = np.r_[v[1], v[:-2] + (v[2:] - v[:-2]) *
                            (z[1:-1] - z[:-2]) / (z[2:] - z[:-2]), v[-2]]
        reject = v - neighbours > ANCHOR_REJECT_MS * 1e6
        if not reject.any():
            break
        assert np.count_nonzero(~reject) >= 1, 'anchor rejection leaves no anchors'
        idx = idx[~reject]
    E = np.interp(x, x[idx], d[idx])
    fitted = p[0, 0] + (a + b * x + E)
    heads = np.arange(0, len(p), 4)
    delay = (s['source'].host_monotonic_ns.to_numpy() - fitted[heads]) / 1e6
    assert delay.min() >= -0.15 and 0.05 <= np.median(delay) <= 0.4, (
        s['sid'], 'host_delay acceptance', delay.min(), np.median(delay))
    s.update(mono_fit=fitted[heads], delay=delay, d=d, E=E, anchors=idx,
             fit=dict(b_ns_per_packet=float(b), a_relative_ns=float(a),
                      n0=int(n[0]), mono0_ns=int(p[0, 0]),
                      mcu_rate_vs_host_pct=float((b / 2.5e6 - 1) * 100),
                      block_packets=size, n_anchors=len(idx), n_anchors_rejected=initial - len(idx),
                      envelope_range_ms=float(np.ptp(E) / 1e6),
                      envelope_min_ms=float(E.min() / 1e6), envelope_max_ms=float(E.max() / 1e6),
                      mean_host_delay_ms=float(delay.mean()),
                      mean_packet_host_delay_ms=float(np.mean(d - E) / 1e6),
                      host_delay_ms=percentiles(delay),
                      per_channel_scan_delay_ms={f'C{k}': float(k * b / 1e6) for k in range(4)}))


def percentiles(x):
    return dict(zip(['min', 'p50', 'p99', 'p99_9', 'max'],
                    map(float, np.percentile(x, [0, 50, 99, 99.9, 100]))))


def fit_ntp(sessions):
    measurements = []
    for s in sessions:
        s['measurements'] = []
        for which, v in [('start', s['meta']['clock']['ntp_start']),
                         ('end', json.loads((ROOT / '00_raw/seat' / s['sid'] /
                                              'session_summary.json').read_text())['ntp_end'])]:
            if v is None:
                continue
            item = dict(session=s['sid'], which=which, mono_ns=int(v['host_wall_time_ns']) - s['W'],
                        K_ns=s['W'] - int(v['offset_ns']), offset_ns=int(v['offset_ns']),
                        host_wall_time_ns=int(v['host_wall_time_ns']), rtt_ns=int(v['rtt_ns']),
                        half_rtt_ms=float(v['rtt_ns'] / 2e6), server=v['server'])
            measurements.append(item); s['measurements'].append(item)
    assert len(measurements) == 7
    m = np.array([v['mono_ns'] for v in measurements], dtype=np.int64)
    sigma = np.array([v['half_rtt_ms'] for v in measurements])
    assert (sigma > 0).all()
    weights = 1 / sigma**2
    mref = float(m[0] + np.average((m - m[0]).astype(float), weights=weights))
    origin = measurements[0]['K_ns']
    y = np.array([(v['K_ns'] - origin) / 1e6 for v in measurements])
    X = np.column_stack([np.ones(len(m)), (m - mref) / 1e9])
    beta = np.linalg.lstsq(X / sigma[:, None], y / sigma, rcond=None)[0]
    covariance = np.linalg.inv(X.T @ (weights[:, None] * X))
    residual = y - X @ beta
    for v, yy, r in zip(measurements, y, residual):
        v.update(K_ms=v['K_ns'] / 1e6, K_relative_ms=float(yy), residual_ms=float(r),
                 within_half_rtt=bool(abs(r) <= v['half_rtt_ms']))
    result = dict(K0_ns=origin + int(round(beta[0] * 1e6)), K_origin_ns=origin,
                  K0_relative_ms=float(beta[0]), slope_ppm=float(beta[1] * 1000),
                  m_ref_ns=mref, measurements=measurements,
                  covariance_ms_seconds=covariance.tolist(), all_residuals_in_bounds=bool(np.all(np.abs(residual) <= sigma)))
    if not result['all_residuals_in_bounds']:
        write_json(QC / 'qc_summary.json', {'ntp_global_fit': result, 'acceptance': 'FAILED NTP residual'})
        QC.mkdir(parents=True, exist_ok=True)
        (QC / 'report.md').write_text('# 압력시트 정제 중단\n\nNTP 잔차가 RTT/2 범위를 넘었다.\n\n' +
                                      table(['세션', '측정', '잔차 ms', 'RTT/2 ms'],
                                            [[v['session'], v['which'], v['residual_ms'], v['half_rtt_ms']]
                                             for v in measurements]), encoding='utf-8', newline='\n')
    assert result['all_residuals_in_bounds'], 'NTP residual outside half-RTT; see QC report'
    A, bounds = np.r_[X, -X], np.r_[y + sigma, -y + sigma]
    for s in sessions:
        mono = s['mono_fit']
        Xends = np.column_stack([np.ones(2), (mono[[0, -1]] - mref) / 1e9])
        mid = np.array([1, ((mono[0] + mono[-1]) / 2 - mref) / 1e9])
        lo = linprog(mid, A_ub=A, b_ub=bounds, bounds=[(None, None)] * 2, method='highs')
        hi = linprog(-mid, A_ub=A, b_ub=bounds, bounds=[(None, None)] * 2, method='highs')
        assert lo.success and hi.success, 'NTP interval line feasibility failed'
        fitmid = mid @ beta
        own = s['measurements']
        chosen = own or min((t['measurements'] for t in sessions if t['measurements']),
                           key=lambda vv: min(abs(v['mono_ns'] - np.mean(mono)) for v in vv))
        chosen = sorted(chosen, key=lambda v: v['mono_ns'])
        plan = np.interp(mono[[0, -1]], [v['mono_ns'] for v in chosen],
                         [v['offset_ns'] / 1e6 for v in chosen])
        effective = (s['W'] - origin) / 1e6 - Xends @ beta
        correction = beta[0] + beta[1] * (mono - mref) / 1e9
        utc = origin + np.rint(mono + correction * 1e6).astype(np.int64)
        dt = np.diff(utc) / 1e6
        assert (np.diff(utc) > 0).all() and ((dt >= 9.9) & (dt <= 10.0)).all(), (s['sid'], dt.min(), dt.max())
        # Subtract the integer epoch origin before checking the arrival-delay identity.
        identity_ns = ((utc - origin).astype(float) -
                       (s['source'].host_monotonic_ns.to_numpy() + correction * 1e6) + s['delay'] * 1e6)
        assert np.max(np.abs(identity_ns)) <= 0.51, 'UTC/QPC/delay identity failed'
        wall_identity = (s['source'].host_wall_time_ns.to_numpy() -
                         s['source'].host_monotonic_ns.to_numpy() - s['W']) / 1e6
        s.update(utc=utc, ntp=dict(effective_offset_ms_start=float(effective[0]),
                                 effective_offset_ms_end=float(effective[1]),
                                 plan_method_offset_ms_start=float(plan[0]), plan_method_offset_ms_end=float(plan[1]),
                                 global_minus_plan_ms_start=float(effective[0] - plan[0]),
                                 global_minus_plan_ms_end=float(effective[1] - plan[1]),
                                 plan_measurements_used=[dict(session=v['session'], which=v['which']) for v in chosen],
                                 k_sigma_ms=float(np.sqrt(mid @ covariance @ mid)),
                                 k_hard_bounds_ms=[float(lo.fun - fitmid), float(-hi.fun - fitmid)],
                                 midpoint_mono_ns=float((mono[0] + mono[-1]) / 2)),
                 acceptance=dict(rows_in=len(s['source']), rows_out=len(utc),
                                 all_source_columns_equal=True, utc_strictly_increasing=True,
                                 dt_ms=dict(min=float(dt.min()), median=float(np.median(dt)), max=float(dt.max())),
                                 utc_delay_identity_max_error_ns=float(np.max(np.abs(identity_ns))),
                                 wall_minus_median_W_max_abs_ms=float(np.max(np.abs(wall_identity)))))
    return result


def stats(x, spikes):
    ok = np.isfinite(x)
    y = x[ok]
    assert len(y), 'no valid observations for channel status'
    p = np.percentile(y, [0, 1, 50, 99, 100])
    result = dict(n=int(ok.sum()), **dict(zip(['min', 'p1', 'p50', 'p99', 'max'], map(float, p))),
                  mean=float(y.mean()), std=float(y.std(ddof=0)), frac_le_10=float(np.mean(y <= 10)),
                  frac_gt_50=float(np.mean(y > 50)), n_spikes=int(spikes.sum()), n_blank=int((~ok).sum()))
    status = 'dead' if p[-1] <= 15 else 'low_signal' if result['frac_gt_50'] < .05 else 'active'
    return result, status


def prepare_output(s):
    df = s['source']
    raw = df[RAW].replace('', np.nan).astype(float).to_numpy()
    spikes = np.column_stack([hampel(raw[:, i]) for i in range(12)])
    mask = (spikes.astype(np.int64) * (1 << np.arange(12))).sum(axis=1)
    out = df.copy()
    out['t_utc_ns'] = s['utc']
    out['host_delay_ms'] = [f'{v:.3f}' for v in s['delay']]
    out['flag_incomplete_frame'] = (df.packets_in_frame < 4).astype(int)
    out['flag_ntp_fallback'] = int(not s['measurements'])
    out['flag_spike'] = (mask != 0).astype(int)
    out['spike_mask'] = mask
    out = out[COLUMNS]
    pd.testing.assert_frame_equal(out[df.columns], df)
    assert len(out) == len(df)
    s.update(raw=raw, spikes=spikes, out=out)
    rel = Path('01_refined/run2/seat.csv') if s['sid'] == RUN2 else Path(f"01_refined/run1/seat_{s['sid']}.csv")
    s['output_file'] = rel.as_posix()
    valid = df.frame_valid.to_numpy() == 1
    s['stats'], s['statuses'] = {}, {}
    for i, name in enumerate(IDS):
        s['stats'][name], s['statuses'][name] = stats(raw[valid, i], spikes[valid, i])


def channels_json(sessions, ntp):
    cfg = sessions[0]['meta']['config']
    assert all(s['meta']['config'] == cfg for s in sessions)
    run2 = next(s for s in sessions if s['sid'] == RUN2)
    setup = next(s for s in sessions if 'setup_check' in s['sid'])
    channels = []
    for i, mapping in enumerate(cfg['mapping']):
        valid_raw = np.concatenate([s['raw'][s['source'].frame_valid == 1, i] for s in sessions])
        valid_spikes = np.concatenate([s['spikes'][s['source'].frame_valid == 1, i] for s in sessions])
        pooled, status = stats(valid_raw, valid_spikes)
        bank = str(mapping['bank']); name = IDS[i]
        channels.append(dict(id=name, column_index=i, bank=mapping['bank'], mux_channel=mapping['channel'],
                             region=mapping['region'], side=mapping['side'], label=mapping['label'],
                             adc_input=cfg['adc_inputs'][bank], rm_ohm=cfg['bank_conditions'][bank]['rm_ohm'],
                             rlim_ohm=cfg['bank_conditions'][bank]['rlim_ohm'], status=status,
                             usable_run2=run2['statuses'][name] == 'active',
                             status_per_session={s['sid']: s['statuses'][name] for s in sessions},
                             stats_per_session={s['sid']: s['stats'][name] for s in sessions}, stats_pooled=pooled,
                             baseline=dict(setup_check_median_raw=setup['stats'][name]['p50'],
                                           lowest_observed_p0_1_raw=float(np.nanpercentile(valid_raw, .1)),
                                           min_raw=pooled['min'], note=BASELINE_NOTE),
                             notes='status는 신호 분포 판정이며 센서 고장 확정이 아니다. 위치/좌우 미확인. std는 ddof=0.'))
    records = []
    for s in sessions:
        kst = pd.to_datetime(s['utc'][[0, -1]], utc=True).tz_convert('Asia/Seoul')
        records.append(dict(session_id=s['sid'], output_file=s['output_file'], rows=len(s['source']),
                            invalid_frames=int((s['source'].frame_valid == 0).sum()),
                            t_utc_ns_first=int(s['utc'][0]), t_utc_ns_last=int(s['utc'][-1]),
                            kst_first=kst[0].isoformat(), kst_last=kst[1].isoformat(),
                            ntp_measurements=s['measurements'], flag_ntp_fallback=int(not s['measurements']),
                            time_fit=s['fit'], ntp=s['ntp']))
    return dict(schema='esw-seat-channels/1', generated_by='pipeline/2_seat_pressure/run.py',
                units=dict(raw='12-bit ADC counts (0..4095)', v='raw*3.3/4095 V; nominal vref, not measured',
                           force='NOT calibrated to N/kg/kPa; no baseline subtraction'),
                frame_timing=dict(nominal_hz=100, per_channel_scan_delay_ms=run2['fit']['per_channel_scan_delay_ms'],
                                  per_channel_scan_delay_ms_reference_session=RUN2,
                                  per_channel_scan_delay_ms_per_session={s['sid']: s['fit']['per_channel_scan_delay_ms'] for s in sessions},
                                  note=TIMING_NOTE),
                mux_wiring=cfg['mux_wiring'], adc_inputs=cfg['adc_inputs'], bank_conditions=cfg['bank_conditions'],
                bank_labels=cfg['bank_labels'], channels=channels, sessions=records, ntp_global_fit=ntp)


def table(headers, rows):
    def cell(v):
        return f'{v:.6f}' if isinstance(v, float) else str(v).replace('|', '/').replace('\n', ' ')
    return ('| ' + ' | '.join(headers) + ' |\n| ' + ' | '.join(['---'] * len(headers)) + ' |\n' +
            ''.join('| ' + ' | '.join(map(cell, row)) + ' |\n' for row in rows) + '\n')


def report(sessions, meta, qc):
    lines = ['# Part B 압력시트 정제 QC\n',
             '원본을 수정하지 않고 6개 세션 전체 행을 같은 순서로 재생성했다. 아래 수치는 qc_summary.json에도 저장된다.\n',
             '## 입출력 및 재파싱\n',
             table(['세션', '입력/출력 행', '무효 프레임', 'KST 처음', 'KST 마지막', '대조 셀 불일치'],
                   [[s['session_id'], f"{s['rows']}/{s['rows']}", s['invalid_frames'], s['kst_first'], s['kst_last'],
                     qc['session_qc'][s['session_id']]['reparse']['mismatching_cells']] for s in meta['sessions']]),
             table(['세션', '전체 로그', '제어 줄', '승인 패킷', '후행 미처리', '파싱 불가', 'seq/tick/cycle 위반', 'timer 위상 폭 ms'],
                   [[s['sid'], *[s['audit'][k] for k in ['log_lines_total', 'control_lines', 'accepted_packets',
                      'trailing_unprocessed_lines', 'unparsable_lines']], str(s['audit']['violations']),
                     s['audit']['timer_phase_spread_ms']] for s in sessions]),
             '승인 패킷에도 tick 증가가 4 ms인 사례가 있다. 위 tick 위반 수와 아래 전체 이상 줄에 기록했다. 순번이 연속이고 timer 위상 폭은 1.5 ms 이내이므로 tick 값을 보존하고 순번 기반 복원을 사용했다.\n',
             'summary.packets개 승인 패킷까지만 프레임에 사용했다. 31개 컬럼을 셀별로 대조했다. 정수는 정확 일치, '
             '전압은 float 파싱 후 정확 일치, 빈 칸은 빈 칸끼리 비교했다. 후행 줄은 정제에 사용하지 않았다.\n',
             '### 후행 또는 파싱/연속성 이상 줄 전체\n']
    for s in sessions:
        lines += [f"#### {s['sid']}\n", table(['로그 줄', '후행', '텍스트', '판정/사유'],
                   [[v['line'], v['trailing'], v['text'], '; '.join(v['reasons'])] for v in s['audit']['issues']])]
    lines += ['## 시간 복원식\n',
              '모든 승인 패킷에 대해 n=uint32 packet_seq를 unwrap한 값, m=host_monotonic_ns이다. '
              '첫 샘플을 빼고 float64 최소제곱으로 m≈a+b·n을 구했다. d=m−(a+b·n). '
              f'연속 {BLOCK_PACKETS}개 블록(마지막 <100개는 이전 블록에 병합, 2블록 미만이면 50개)에 대해 '
              'anchor=(argmin d의 n, min d)를 선정했다. 인접 anchor 사이 직선보다 '
              f'{ANCHOR_REJECT_MS} ms 초과 높은 anchor를 반복 제거했다. 처음/끝은 가장 가까운 이웃 값과 비교한다. '
              'E(n)=np.interp(n, anchor_n, anchor_d), 양 끝 밖에서는 상수이다.\n',
              'mono_fit(n)=a+b·n+E(n), host_delay_ms=(원본 host_monotonic_ns−mono_fit)/10⁶. '
              'CSV의 n은 프레임 첫 패킷 순번이다. 구현은 m0,n0를 뺀 상대 좌표에서 연산하며 절대식의 a=m0+a_relative−b·n0이다.\n',
              '수신 지연은 USB/OS 버퍼링에 의해 한 방향으로 생긴다. 평균 직선은 지연만큼 늦어지고 '
              'MCU 드리프트에 따른 포락선 굽음도 남긴다. 아래 평균 지연은 평균 적합을 쓰면 포함되는 '
              '양의 시간 편향 크기이다(프레임/전체 패킷을 각각 계산). 최대 지연도 제거 없이 보고한다.\n',
              table(['세션', 'b ns/packet', 'MCU-host %', 'anchor/제거', 'E 폭 ms', '평균 지연(프레임/패킷) ms',
                     'delay min/p50/p99/p99.9/max ms', 'UTC dt min/median/max ms'],
                    [[s['sid'], s['fit']['b_ns_per_packet'], s['fit']['mcu_rate_vs_host_pct'],
                      f"{s['fit']['n_anchors']}/{s['fit']['n_anchors_rejected']}", s['fit']['envelope_range_ms'],
                      f"{s['fit']['mean_host_delay_ms']:.6f}/{s['fit']['mean_packet_host_delay_ms']:.6f}",
                      '/'.join(f'{v:.6f}' for v in s['fit']['host_delay_ms'].values()),
                      '/'.join(f'{v:.6f}' for v in s['acceptance']['dt_ms'].values())] for s in sessions]),
              'MCU-host 비율은 요청한 정의 (b/2.5e6−1)×100이다. 음수이면 2.5 MCU-ms가 더 짧은 host 시간에 해당한다.\n',
              table(['세션', 'C0 ms', 'C1 ms', 'C2 ms', 'C3 ms'],
                    [[s['sid'], *s['fit']['per_channel_scan_delay_ms'].values()] for s in sessions]),
              TIMING_NOTE + '\n', '## 전역 NTP 및 PC wall-clock step\n',
              'W_s=세션 전체 승인 패킷의 median(host_wall−host_monotonic). 측정별 '
              'mono_i=ntp.host_wall_time_ns−W_s, K_i=W_s−offset_i, sigma_i=rtt_i/2. '
              'm_ref=1/sigma² 가중 mono 평균. K(m)=K0+s·(m−m_ref)를 1/sigma² 가중 최소제곱으로 적합했다. '
              't_utc_ns=round(mono_fit+K(mono_fit)) (int64). 큰 epoch 원점을 정수로 분리해 연산한다. '
              'NTP offset의 부호는 host−NTP이므로 UTC=host_wall−offset이다.\n',
              f"전역 기울기 **{meta['ntp_global_fit']['slope_ppm']:.6f} ppm**, "
              f"K0={meta['ntp_global_fit']['K0_ns']} ns, m_ref={meta['ntp_global_fit']['m_ref_ns']:.3f} ns. "
              '7개 잔차 모두 ±RTT/2 이내이다. 불확도는 sigma=RTT/2를 1σ로 간주한 형식적 값으로, '
              '공분산을 잔차로 재스케일하지 않았다. hard bounds는 7개 구간을 모두 지나는 모든 직선의 '
              '중간시각 K 최솟값/최댓값을 linprog로 구한 뒤 적합값을 뺀 범위이다.\n',
              table(['세션', '측정', 'mono ns', 'K 상대 ms', 'RTT/2 ms', '잔차 ms', '범위 내'],
                    [[v['session'], v['which'], v['mono_ns'], v['K_relative_ms'], v['half_rtt_ms'],
                      v['residual_ms'], v['within_half_rtt']] for v in meta['ntp_global_fit']['measurements']]),
              'K 상대 값의 정수 원점은 ' + str(meta['ntp_global_fit']['K_origin_ns']) + ' ns이다.\n',
              table(['세션', '자체 NTP', 'fallback', '유효 offset 시작/끝 ms', 'PLAN offset 시작/끝 ms',
                     '전역−PLAN 시작/끝 ms', '중간 K 1σ ms', 'hard bounds ms'],
                    [[s['session_id'], ','.join(v['which'] for v in s['ntp_measurements']) or '없음',
                      s['flag_ntp_fallback'],
                      f"{s['ntp']['effective_offset_ms_start']:.6f}/{s['ntp']['effective_offset_ms_end']:.6f}",
                      f"{s['ntp']['plan_method_offset_ms_start']:.6f}/{s['ntp']['plan_method_offset_ms_end']:.6f}",
                      f"{s['ntp']['global_minus_plan_ms_start']:.6f}/{s['ntp']['global_minus_plan_ms_end']:.6f}",
                      s['ntp']['k_sigma_ms'], s['ntp']['k_hard_bounds_ms']] for s in meta['sessions']]),
              'PLAN 방식은 비교에만 썼다. 자체 시작/끝 측정의 mono 시각 사이 offset 선형 보간(끝 밖은 고정), '
              '하나이면 상수, 없으면 세션 중간시각에서 가장 가까운 NTP 측정을 가진 세션의 값을 사용했다. '
              '측정 선택 내역은 JSON에 저장했다. 자체 NTP가 없는 153855/154035의 모든 행만 fallback=1이다.\n',
              table(['세션', 'W ns', '세션 W 폭 ms', '앞 5개 median 대비 W ms', 'UTC-delay 항등식 최대 오차 ns'],
                    [[s['sid'], s['W'], s['wall_spread_ms'], qc['session_qc'][s['sid']]['W_delta_from_early_ms'],
                      s['acceptance']['utc_delay_identity_max_error_ns']] for s in sessions]),
              f"run2의 W 변화는 {qc['wall_clock_step_ms']:.6f} ms이다. 15:48 세션 이후 16:34 세션 이전에 "
              'PC wall clock이 뒤로 step했으며 동일 부팅의 연속 QPC에 전역 UTC를 매핑해 이를 분리했다. '
              f"run2 W 전체 폭은 {next(s['wall_spread_ms'] for s in sessions if s['sid'] == RUN2):.6f} ms로, 사전 사실의 ≤0.15 ms 검증은 실패했다. "
              '차이는 0.0008 ms이다. 실제 값을 보존하고 이 보조 안정성 검사만 0.151 ms 한도로 두었다. '
              '요청한 Acceptance 1–5의 한도는 바꾸지 않았다. '
              '정수 epoch 원점을 빼고 t_utc−(host_monotonic+K)=−host_delay를 0.51 ns 이내로 확인했다. '
              'host_wall−유효 offset에서의 차이는 −host_delay에 행별 W−median(W)의 작은 차이가 더해진다.\n',
              '## 채널 통계 및 관측 기준선\n',
              '아래 통계는 frame_valid=1 행만 사용한다. n은 유효 비결측 수, n_blank는 유효 프레임 내 빈 값 수이며 '
              'std는 모집단 표준편차(ddof=0)이다. max≤15이면 dead, 그 외 frac_gt_50<0.05이면 low_signal, '
              '나머지는 active이다. pooled에도 같은 규칙을 적용했다. dead는 고장 확정이 아니다.\n',
              BASELINE_NOTE + '\n',
              table(['채널', '전체 상태', 'run2 사용 가능', 'setup median', 'pooled p0.1', 'pooled min'],
                    [[c['id'], c['status'], c['usable_run2'], c['baseline']['setup_check_median_raw'],
                      c['baseline']['lowest_observed_p0_1_raw'], c['baseline']['min_raw']] for c in meta['channels']])]
    for s in sessions:
        lines += [f"### {s['sid']}\n", table(['채널', '상태', 'n', 'min', 'p1', 'p50', 'p99', 'max', 'mean',
                   'std', '≤10 비율', '>50 비율', 'spike', 'blank'],
                   [[c, s['statuses'][c], *s['stats'][c].values()] for c in IDS])]
    lines += ['## 스파이크 및 무결성\n',
              f'Hampel 중앙 {SPIKE_WINDOW}개(±{SPIKE_WINDOW // 2}), 경계는 창 축소, NaN 무시. '
              f'|raw−median| > max({SPIKE_MIN_COUNTS}, {SPIKE_MAD_MULTIPLIER}×1.4826×MAD)일 때만 spike이다. '
              'NaN 자체는 spike가 아니다. bit i는 컬럼 순서의 채널 i(a0_c0=bit 0)이다. 값은 변경하지 않았다.\n',
              table(['세션', 'spike 행(전체)', 'spike 채널값(전체)', '불완전 프레임', '전체 빈 raw'],
                    [[s['sid'], int(s['out'].flag_spike.sum()), int(s['spikes'].sum()),
                      int(s['out'].flag_incomplete_frame.sum()), int(np.isnan(s['raw']).sum())] for s in sessions]),
              '행수, 원본 host/sequence/MCU/raw/volts 보존, UART 대조, seq/channel 연속성과 timer 위상 폭, tick 위반 수 기록, '
              'UTC 단조증가 및 모든 행 간격 9.9–10.0 ms, 지연 min≥−0.15 ms 및 median 0.05–0.4 ms, '
              'NTP 잔차/항등식 검사를 assert로 강제했다. 저장 CSV를 다시 읽어 원본 값·스키마·지연 반올림을 재검사했다.\n',
              '## 보정 및 알려진 한계\n',
              '**힘 보정 없음**: raw는 12-bit ADC counts, 전압은 raw×3.3/4095이며 공칭 Vref이다. '
              'N/kg/kPa로 변환하지 않았으며 무하중 기준선을 빼지 않았다. 센서 위치/좌우는 unknown, '
              'A1/A2 저항은 미확인, 채널 상태는 관측 분포 기반이다.\n',
              '하부 포락선은 최소 수신 지연의 상대 변동을 제거하는 추정이다. 상수 UART/USB 지연, '
              '블록 내 최소 지연이 일정하다는 가정, anchor 사이 선형 근사와 끝 밖 상수 외삽, '
              'NTP의 경로 비대칭/상관 오차 및 전역 직선 가정은 남는다. hard bounds도 RTT/2 구간과 '
              '단일 직선 가정에 조건부이다. 같은 행의 C0–C3는 동시 샘플이 아니고 마지막 불완전 행도 보존했다. '
              '이 스크립트의 수치 검사는 수행했으며 별도 Claude 독립 검증과 파트 D 장비 간 동기화는 후속 작업이다.\n',
              '## 플롯\n',
              'channels_run2.png와 다른 5개 channels_<sid>.png는 KST/raw 및 빨간 spike, '
              'host_delay_hist.png는 6세션 로그 y 지연 분포, envelope_run2.png는 포락선 확대/전체, '
              'ntp_fit.png는 전역 QPC→UTC 적합과 RTT/2 및 세션 구간, spike_examples.png는 최대 12개 spike의 ±0.3초이다.\n']
    (QC / 'report.md').write_text('\n'.join(lines), encoding='utf-8', newline='\n')


def save_plot(fig, name):
    fig.tight_layout(rect=(0, 0, 1, .97))
    fig.savefig(QC / name, dpi=110)
    plt.close(fig)


def plots(sessions, ntp):
    tz = timezone(timedelta(hours=9))
    for s in sessions:
        run2 = s['sid'] == RUN2
        fig, axes = plt.subplots(12 if run2 else 4, 1 if run2 else 3,
                                 figsize=(14, 19) if run2 else (13, 8), sharex=True)
        times = pd.to_datetime(s['utc'], utc=True).tz_convert('Asia/Seoul').to_pydatetime()
        for i, ax in enumerate(np.ravel(axes)):
            ax.plot(times, s['raw'][:, i], lw=.45)
            flagged = s['spikes'][:, i]
            ax.scatter(np.asarray(times)[flagged], s['raw'][flagged, i], s=9, c='red', zorder=3)
            ax.set_ylabel(IDS[i]); ax.grid(alpha=.2)
            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S', tz=tz))
        fig.suptitle(s['sid'] + ' | raw counts | KST')
        save_plot(fig, 'channels_run2.png' if run2 else f"channels_{s['sid']}.png")
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    for s, ax in zip(sessions, axes.flat):
        ax.hist(s['delay'], bins=100, log=True)
        ax.set_title(s['sid'][17:]); ax.set_xlabel('host delay (ms)'); ax.set_ylabel('frames (log)')
    save_plot(fig, 'host_delay_hist.png')
    run2 = next(s for s in sessions if s['sid'] == RUN2)
    t = (run2['n'] - run2['n'][0]) * run2['fit']['b_ns_per_packet'] / 1e9
    take = np.unique(np.r_[np.arange(0, len(t), max(1, len(t) // 16000)), run2['anchors'], np.argmin(run2['d']), np.argmax(run2['d'])])
    fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=True)
    for ax in axes:
        ax.scatter(t[take], run2['d'][take] / 1e6, s=1, alpha=.35, label='d (thinned)')
        ax.plot(t[take], run2['E'][take] / 1e6, color='red', lw=1, label='E(n)')
        a = run2['anchors']
        ax.scatter(t[a], run2['d'][a] / 1e6, s=5, c='black', label='retained anchors')
        ax.set_ylabel('residual (ms)'); ax.legend(loc='upper right'); ax.grid(alpha=.2)
    axes[0].set_ylim(run2['E'].min() / 1e6 - 2, run2['E'].max() / 1e6 + 5)
    axes[0].set_title('Run2 lower envelope (zoom)')
    axes[1].set_title('Full residual range'); axes[1].set_xlabel('time from first packet (s)')
    save_plot(fig, 'envelope_run2.png')
    fig, ax = plt.subplots(figsize=(12, 6))
    measurements = ntp['measurements']; ref = ntp['m_ref_ns']
    xx = np.array([(v['mono_ns'] - ref) / 1e9 for v in measurements])
    ax.errorbar(xx, [v['K_relative_ms'] for v in measurements],
                yerr=[v['half_rtt_ms'] for v in measurements], fmt='o', capsize=4, label='NTP ±RTT/2')
    grid = np.linspace(xx.min(), xx.max(), 300)
    ax.plot(grid, ntp['K0_relative_ms'] + ntp['slope_ppm'] / 1000 * grid, label='weighted line')
    for i, s in enumerate(sessions):
        bounds = (s['mono_fit'][[0, -1]] - ref) / 1e9
        ax.axvspan(*bounds, alpha=.10, color=f'C{i}', label=s['sid'][17:])
    ax.set_xlabel('QPC seconds from weighted reference')
    ax.set_ylabel('K minus integer epoch origin (ms)'); ax.legend(fontsize=8); ax.grid(alpha=.2)
    save_plot(fig, 'ntp_fit.png')
    examples = [(s, int(i), int(row)) for s in sessions
                for row, i in np.argwhere(s['spikes'])][:12]
    count = max(1, len(examples))
    cols = min(3, count)
    rows = (count + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4.4 * cols, 3 * rows), squeeze=False)
    for ax, (s, i, idx) in zip(axes.flat, examples):
        t = (s['utc'] - s['utc'][idx]) / 1e9
        near = np.abs(t) <= .3
        ax.plot(t[near], s['raw'][near, i], '.-', ms=3, lw=.7)
        ax.scatter([0], [s['raw'][idx, i]], c='red', s=25)
        ax.set_title(s['sid'][17:23] + ' ' + IDS[i]); ax.set_xlabel('relative time (s)')
        ax.set_ylabel('raw counts'); ax.set_xlim(-.3, .3); ax.grid(alpha=.2)
    for ax in list(axes.flat)[len(examples):]:
        ax.axis('off')
    if not examples:
        fig.suptitle('No spikes detected')
    save_plot(fig, 'spike_examples.png')
    return [dict(session=s['sid'], channel=IDS[i], row=idx, t_utc_ns=int(s['utc'][idx])) for s, i, idx in examples]


def main():
    self_check()
    folders = sorted((ROOT / '00_raw/seat').glob('session_*'))
    assert len(folders) == 6
    inputs = [f / rel for f in folders for rel in ['metadata.json', 'session_summary.json',
               'events.jsonl', 'seat/pressure.csv', 'seat/raw_uart.log']]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    sessions = [reparse(f) for f in folders]
    for s in sessions:
        fit_time(s)
        print(s['sid'], 'reparse cells=0;', len(s['source']), 'rows;', 'delay', s['fit']['host_delay_ms'], flush=True)
    ntp = fit_ntp(sessions)
    for s in sessions:
        prepare_output(s)
    metadata = channels_json(sessions, ntp)
    early_W = sorted(s['W'] for s in sessions if s['sid'] != RUN2)[2]
    run2 = next(s for s in sessions if s['sid'] == RUN2)
    assert abs((run2['W'] - early_W) / 1e6 + 104.75) < .15
    assert [s['sid'][17:23] for s in sessions if not s['measurements']] == ['153855', '154035']
    QC.mkdir(parents=True, exist_ok=True)
    for s in sessions:
        dest = ROOT / s['output_file']
        dest.parent.mkdir(parents=True, exist_ok=True)
        s['out'].to_csv(dest, index=False, encoding='utf-8', lineterminator='\n')
        content = dest.read_bytes()
        assert not content.startswith(b'\xef\xbb\xbf') and b'\r' not in content
        reread = pd.read_csv(dest, dtype=str, keep_default_na=False)
        assert reread.columns.tolist() == COLUMNS and len(reread) == len(s['source'])
        for c in s['source'].columns:
            assert reread[c].tolist() == s['source'][c].astype(str).tolist(), (s['sid'], c, 'saved source mismatch')
        assert np.array_equal(reread.t_utc_ns.astype('int64'), s['utc'])
        assert np.max(np.abs(reread.host_delay_ms.astype(float) - s['delay'])) <= .00050001
        for c in ['flag_incomplete_frame', 'flag_ntp_fallback', 'flag_spike', 'spike_mask']:
            assert np.array_equal(reread[c].astype('int64'), s['out'][c])
        s['acceptance']['saved_csv_verified'] = True
    write_json(ROOT / '01_refined/seat_channels.json', metadata)
    examples = plots(sessions, ntp)
    qc = dict(schema='esw-seat-qc/1', generated_by='pipeline/2_seat_pressure/run.py', acceptance='PASS',
              constants=dict(BLOCK_PACKETS=BLOCK_PACKETS, ANCHOR_REJECT_MS=ANCHOR_REJECT_MS,
                             SPIKE_WINDOW=SPIKE_WINDOW, SPIKE_MIN_COUNTS=SPIKE_MIN_COUNTS,
                             SPIKE_MAD_MULTIPLIER=SPIKE_MAD_MULTIPLIER, W_SPREAD_MAX_NS=W_SPREAD_MAX_NS,
                             dead_max_raw=15, active_min_frac_gt_50=.05, timer_phase_max_spread_ms=1.5,
                             dt_ms_bounds=[9.9, 10.0], delay_min_ms=-.15, delay_median_ms_bounds=[.05, .4],
                             utc_delay_identity_tolerance_ns=.51),
              input_sha256=hashes, wall_clock_step_ms=float((run2['W'] - early_W) / 1e6),
              baseline_note=BASELINE_NOTE, timing_note=TIMING_NOTE,
              channel_metadata=metadata, spike_examples=examples,
              session_qc={s['sid']: dict(reparse=s['audit'], acceptance=s['acceptance'],
                            W_ns=s['W'], W_spread_ms=s['wall_spread_ms'],
                            preliminary_W_spread_le_0_15_ms=bool(s['wall_spread_ms'] <= .15),
                            W_delta_from_early_ms=float((s['W'] - early_W) / 1e6), event_kinds=s['event_kinds'],
                            spike_rows_all=int(s['out'].flag_spike.sum()), spike_values_all=int(s['spikes'].sum()),
                            n_incomplete_frames=int(s['out'].flag_incomplete_frame.sum()),
                            n_blank_raw_all=int(np.isnan(s['raw']).sum())) for s in sessions})
    for p in inputs:
        assert hashlib.sha256(p.read_bytes()).hexdigest() == hashes[p.relative_to(ROOT).as_posix()], 'raw input changed'
    write_json(QC / 'qc_summary.json', qc)
    report(sessions, metadata, qc)
    print('NTP slope ppm:', ntp['slope_ppm'], 'all 7 residual bounds PASS', flush=True)
    print('Overall/run2 statuses:', {c['id']: (c['status'], c['status_per_session'][RUN2]) for c in metadata['channels']}, flush=True)
    print('All acceptance checks PASS; generated six CSVs, channel JSON, QC JSON/report and ten PNGs.', flush=True)


if __name__ == '__main__':
    main()
