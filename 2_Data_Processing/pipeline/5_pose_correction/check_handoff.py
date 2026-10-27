"""Compare segment handoff bytes with the corrected table (ignoring D suffix)."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def comparable(path, segments):
    with path.open('rb') as f:
        header = next(f)
        columns = next(csv.reader([header.decode('utf-8')]))
        suffix = columns[-2:] == ['t_run_s', 'in_obd_window']
        if ('t_run_s' in columns or 'in_obd_window' in columns) and not suffix:
            raise ValueError(f'Unexpected Part D column placement: {path}')
        segment_col = columns.index('segment_id')
        if suffix:
            header = header.rstrip(b'\n').rsplit(b',', 2)[0] + b'\n'
        output = [header]
        for line in f:
            row = next(csv.reader([line.decode('utf-8')]))
            if segments is None or int(row[segment_col]) in segments:
                output.append(line.rstrip(b'\n').rsplit(b',', 2)[0] + b'\n' if suffix else line)
    return b''.join(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--segments', default='all')
    args = parser.parse_args()
    folders = sorted((ROOT/'_work/5_pose_correction/handoff').glob('seg_[0-9][0-9][0-9]'))
    available = [int(p.name[4:]) for p in folders if (p/'DONE.json').exists()]
    segments = available if args.segments == 'all' else sorted(set(int(s) for s in args.segments.split(',')))
    if not segments or any(s not in available for s in segments):
        parser.error(f'Missing handoff; requested={segments}; available={available}')
    chunks, header = [], None
    for sid in segments:
        folder = ROOT/'_work/5_pose_correction/handoff'/f'seg_{sid:03d}'
        path = folder/f'pose_landmarks_corrected_seg_{sid:03d}.csv'
        payload = path.read_bytes()
        done = json.loads((folder/'DONE.json').read_text(encoding='utf-8'))
        digest = hashlib.sha256(payload).hexdigest()
        if digest != done['csv_sha256']:
            raise SystemExit(f'DONE hash mismatch: {path}')
        current_header, body = payload.split(b'\n', 1)
        if header is None:
            header = current_header+b'\n'
        elif current_header+b'\n' != header:
            raise SystemExit(f'Schema mismatch: {path}')
        chunks.append(body)
    actual = header+b''.join(chunks)
    expected = comparable(ROOT/'01_refined/run2/pose_landmarks_corrected.csv',
                          None if args.segments == 'all' else segments)
    same = actual == expected
    print(json.dumps({'segments': segments, 'handoff_sha256': hashlib.sha256(actual).hexdigest(),
                      'corrected_sha256': hashlib.sha256(expected).hexdigest(), 'byte_identical': same}, indent=2), flush=True)
    raise SystemExit(0 if same else 1)


if __name__ == '__main__':
    main()
