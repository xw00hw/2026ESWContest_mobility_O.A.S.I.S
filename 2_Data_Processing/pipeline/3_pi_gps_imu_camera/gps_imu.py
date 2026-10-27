"""Refine Pi GPS/IMU and run1 fragments; reproduce QC with main()."""
from pathlib import Path
from collections import Counter
from datetime import datetime, timezone, timedelta
import csv
import json
import math
import struct
import re
import tempfile

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

ROOT = Path(__file__).resolve().parents[2]
RUN2 = ROOT / "00_raw/pi_run2/session_run2"
QC = ROOT / "02_qc/3_pi_gps_imu_camera"
KST = timezone(timedelta(hours=9))
TRUST_CORR_LONG = 0.7
TRUST_CORR_YAW = 0.9
TRUST_YAW_UNCERTAINTY_DEG = 5.0
GYRO_STATIONARY_MAX_RADPS = 0.03
SEED = 2026
ENVELOPE_MIN_SAMPLES = 600      # 60 s at 10 Hz
PERIOD_DEVIATION_MAX = 1e-3     # envelope-fit period vs OLS period
IMU_BAUD = 9600                 # session_summary.json workers.imu.baud
TRUST_LEVEL_SD_DEG = 1.0
TRUST_LONG_OFFSET_MPS2 = 0.12
OUTLIER_STOP_DEG = 2.0
GPS_COLUMNS = "t_utc_ns host_monotonic_ns t_source epoch_idx gps_utc gps_utc_ns gps_utc_corrected_ns host_minus_gps_s lat_deg lon_deg alt_msl_m geoid_sep_m east_m north_m up_m speed_mps speed_kmh speed_vtg_kmh course_deg speed_source rmc_status rmc_mode fix_quality gsa_fix_type sats_used sats_in_view hdop pdop vdop has_rmc has_gga has_gsa has_vtg valid flag_leap_offset flag_incomplete_epoch flag_gap_before flag_time_mismatch".split()
INT_GPS = set("t_utc_ns host_monotonic_ns epoch_idx gps_utc_ns gps_utc_corrected_ns fix_quality gsa_fix_type sats_used sats_in_view has_rmc has_gga has_gsa has_vtg valid flag_leap_offset flag_incomplete_epoch flag_gap_before flag_time_mismatch".split())
IMU_PREFIX = "t_utc_ns host_wall_time_ns host_monotonic_ns host_delay_ms sensor_sequence dt_host_s flag_host_burst flag_gap_before flag_ts_reconstructed".split()


def read_csv(path):
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame()
    return pd.read_csv(path, dtype=str, keep_default_na=False).replace("", pd.NA)


def integerize(df, columns):
    for c in columns:
        if c in df:
            df[c] = pd.array([None if pd.isna(v) else int(v) for v in df[c]], dtype="Int64")
    return df


def write_csv(df, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ns = [c for c in df if c.endswith("_ns")]
    assert all(str(df[c].dtype) == "Int64" for c in ns)
    out = df.copy()
    for c in out:
        if out[c].dtype.kind == "f":  # fixed notation, shortest repr that round-trips
            out[c] = [("" if pd.isna(v) else np.format_float_positional(v, trim="-")) for v in out[c]]
    out.to_csv(path, index=False, encoding="utf-8", lineterminator="\n", na_rep="")
    assert not re.search(r"[0-9]e[-+]?[0-9]", path.read_text(encoding="utf-8").split("\n",1)[1]), path
    with path.open(encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            assert all(not row[c] or re.fullmatch(r"-?\d+", row[c]) for c in ns)


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if value is pd.NA:
        return None
    return value


def write_json(value, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(clean_json(value), ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8", newline="\n")


def kst(ns):
    return datetime.fromtimestamp(int(ns)//10**9, KST).isoformat() if ns is not None and not pd.isna(ns) else ""


def distribution(values):
    x = np.asarray(values, dtype=float)
    x = x[np.isfinite(x)]
    if not len(x):
        return {k: None for k in ["n", "median", "mad", "p5", "p50", "p95", "p99", "min", "max"]}
    med = np.median(x)
    return dict(n=len(x), median=med, mad=np.median(abs(x-med)), p5=np.percentile(x,5), p50=med, p95=np.percentile(x,95), p99=np.percentile(x,99), min=x.min(), max=x.max())


def bins60(times, values):
    times = np.asarray(times, dtype=np.int64)
    values = np.asarray(values, dtype=float)
    if not len(times):
        return []
    bins = (times-times[0])//60_000_000_000
    return [dict(t_start_utc_ns=int(times[0])+int(b)*60_000_000_000, t_start_kst=kst(int(times[0])+int(b)*60_000_000_000), median=float(np.median(values[(bins==b)&np.isfinite(values)])) if np.any((bins==b)&np.isfinite(values)) else None, n=int(sum(bins==b))) for b in np.unique(bins)]


def number(s):
    return float(s) if s else None


def coord(s, hemisphere):
    if not s or not hemisphere:
        return None
    dot = s.find(".")
    n = (len(s) if dot < 0 else dot)-2
    return (float(s[:n])+float(s[n:])/60) * (-1 if hemisphere in ("S","W") else 1)


def utc_rmc(date, time):
    if not date or not time:
        return None, None
    yr = int(date[4:6])
    yr += 2000 if yr < 80 else 1900
    whole, _, fraction = time.partition(".")
    base = datetime(yr, int(date[2:4]), int(date[:2]), int(whole[:2]), int(whole[2:4]), tzinfo=timezone.utc)
    ns = int(base.timestamp())*10**9 + int(whole[4:6])*10**9 + int((fraction+"000000000")[:9])
    iso = datetime.fromtimestamp(ns//10**9, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S")+f".{(ns%10**9)//10**6:03d}Z"
    return iso, ns


def parse_nmea(raw):
    try:
        body, declared = raw.strip()[1:].rsplit("*",1)
        check = 0
        for ch in body:
            check ^= ord(ch)
        fields = body.split(",")
        return fields[0], fields[1:], raw[0] in "$!" and check == int(declared,16)
    except (ValueError, IndexError):
        return "", [], False


def gps_epochs(session):
    path = session / "gps/raw_nmea.log"
    counts, valid_counts, collector_counts = Counter(), Counter(), Counter()
    errors, malformed, epochs, recovered = [], [], [], []
    dropped_before_first_epoch = [0]
    current = None
    pending_vtg = None
    def finish():
        if current and ("RMC" in current["parts"] or "GGA" in current["parts"]):
            epochs.append(current)
    if path.exists():
        for line_idx, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines()):
            if not line.strip():
                continue
            try:
                mono_s, wall_s, raw = line.split(" ",2)
                mono, wall = int(mono_s), int(wall_s)
            except ValueError:
                malformed.append(dict(line_idx=line_idx,raw=line))
                continue
            embedded=[m.group(0) for m in re.finditer(r"\$[A-Z]{5},[^$*]*\*[0-9A-Fa-f]{2}",raw) if m.start()>0 and parse_nmea(m.group(0))[2]]
            if not raw.startswith(("$", "!")):
                malformed.append(dict(line_idx=line_idx,host_monotonic_ns=mono,host_wall_time_ns=wall,raw=raw,reason="continuation fragment without NMEA start marker"))
                queue=[]
            else:
                queue=[raw]
            for item in embedded:
                recovered.append(dict(line_idx=line_idx,host_monotonic_ns=mono,host_wall_time_ns=wall,sentence=item))
            queue+=embedded
            for raw in queue:
              syntax = re.fullmatch(r"[!$]([A-Za-z0-9]{2,})((?:,[^*]*)*)(?:\*([0-9A-Fa-f]{2}))?\s*", raw)
              if syntax:
                  collector_counts[syntax.group(1)] += 1
              typ, fields, ok = parse_nmea(raw)
              kind = "RMC" if raw.startswith(("$GPRMC,", "$GNRMC,")) else typ[-3:]
              counts[typ] += 1
              # Invalid RMC marks a burst boundary but supplies no epoch values.
              if kind == "RMC":
                  if pending_vtg is not None and current is not None:
                      current["parts"]["VTG"] = pending_vtg
                  pending_vtg = None
                  finish()
                  current = dict(parts={}, first=None, boundary="RMC")
              if not ok:
                  errors.append(dict(line_idx=line_idx,host_monotonic_ns=mono,host_wall_time_ns=wall,raw=raw))
                  continue
              valid_counts[typ] += 1
              if kind not in ("RMC","VTG","GGA","GSA","GSV","GLL"):
                  continue
              if kind == "VTG" and (current is None or ("GGA" in current["parts"] and mono-current["parts"]["GGA"][0]>500_000_000)):
                  pending_vtg = (mono,wall,fields)
                  continue
              if kind == "GGA":
                  old = current["parts"] if current else {}
                  prior = old.get("GGA")
                  rmc = old.get("RMC")
                  new = current is None or (prior is not None and fields[0] != prior[2][0])
                  if rmc and fields[0] != rmc[2][0] and mono-rmc[0] > 500_000_000:
                      new = True
                  if new:
                      finish()
                      current = dict(parts={},first=None,boundary="GGA")
                  if pending_vtg is not None:
                      current["parts"]["VTG"] = pending_vtg
                      if current["first"] is None:
                          current["first"] = (pending_vtg[0],pending_vtg[1],"VTG")
                      pending_vtg = None
              if current is None:
                  dropped_before_first_epoch[0]+=1
                  continue
              if current["first"] is None:
                  current["first"] = (mono,wall,kind)
              current["parts"][kind] = (mono,wall,fields)
        if pending_vtg is not None and current is not None:
            current["parts"]["VTG"] = pending_vtg
        finish()
    rows, matches = [], []
    for idx, epoch in enumerate(epochs):
        p = epoch["parts"]
        r = p.get("RMC",(None,None,[]))[2]
        g = p.get("GGA",(None,None,[]))[2]
        a = p.get("GSA",(None,None,[]))[2]
        v = p.get("VTG",(None,None,[]))[2]
        def at(fields, i):
            return fields[i] if i < len(fields) else ""
        anchor = (p["RMC"][0],p["RMC"][1],"RMC") if "RMC" in p else epoch["first"]
        mono,wall,source = anchor
        iso,utc = utc_rmc(at(r,8),at(r,0))
        # GGA has no date: UTC stays missing, rather than borrowing a previous date.
        raw_delta = (wall-utc)/1e9 if utc is not None else None
        leap = int(raw_delta is not None and raw_delta < -1)
        corrected = utc-2_000_000_000 if leap else utc
        lat = coord(at(r,2),at(r,3))
        lon = coord(at(r,4),at(r,5))
        lat = lat if lat is not None else coord(at(g,1),at(g,2))
        lon = lon if lon is not None else coord(at(g,3),at(g,4))
        speed = number(at(r,6))
        speed_source = "RMC" if speed is not None else None
        if speed is None and number(at(v,4)) is not None:
            speed, speed_source = number(at(v,4)), "VTG"
        speed = speed*0.514444 if speed is not None else None
        course = number(at(r,7)) if "RMC" in p else number(at(v,0))
        quality = int(at(g,5)) if at(g,5) else None
        rows.append(dict(t_utc_ns=wall,host_monotonic_ns=mono,t_source=source,epoch_idx=idx,gps_utc=iso,gps_utc_ns=utc,gps_utc_corrected_ns=corrected,host_minus_gps_s=(wall-corrected)/1e9 if source=="RMC" and corrected is not None else None,lat_deg=lat,lon_deg=lon,alt_msl_m=number(at(g,8)),geoid_sep_m=number(at(g,10)),speed_mps=speed,speed_kmh=speed*3.6 if speed is not None else None,speed_vtg_kmh=number(at(v,6)),course_deg=course,speed_source=speed_source,rmc_status=at(r,1) or None,rmc_mode=at(r,11) or None,fix_quality=quality,gsa_fix_type=int(at(a,1)) if at(a,1) else None,sats_used=int(at(g,6)) if at(g,6) else None,sats_in_view=int(at(p.get("GSV",(0,0,[]))[2],2)) if at(p.get("GSV",(0,0,[]))[2],2) else None,hdop=number(at(g,7)),pdop=number(at(a,14)),vdop=number(at(a,16)),has_rmc=int("RMC" in p),has_gga=int("GGA" in p),has_gsa=int("GSA" in p),has_vtg=int("VTG" in p),valid=int(("RMC" not in p or at(r,1)=="A") and (quality or 0)>=1 and lat is not None and lon is not None),flag_leap_offset=leap,flag_incomplete_epoch=int("RMC" not in p or "GGA" not in p),flag_gap_before=int(idx>0 and wall-rows[-1]["t_utc_ns"]>1_500_000_000),flag_time_mismatch=int(bool(at(r,0) and at(g,0) and number(at(r,0))!=number(at(g,0))))))
        matches.append({kind: int(p[kind][0]) for kind in ("RMC","GGA") if kind in p})
    known=[row["flag_leap_offset"] if row["gps_utc_ns"] is not None else None for row in rows]
    for i,row in enumerate(rows):
        if known[i] is None:
            before=next((k for k in reversed(known[:i]) if k is not None),None)
            after=next((k for k in known[i+1:] if k is not None),None)
            sides=[k for k in (before,after) if k is not None]
            row["flag_leap_offset"]=int(bool(sides) and all(sides))
    # Construct integer arrays before DataFrame inference can create float ns values.
    df = pd.DataFrame({c: pd.array([row.get(c) for row in rows],dtype="Int64") if c in INT_GPS else [row.get(c) for row in rows] for c in GPS_COLUMNS})
    rmc_count = sum(v for k,v in valid_counts.items() if k.endswith("RMC"))
    assert len(df)==len(epochs)==rmc_count+int(((df.has_gga==1)&(df.has_rmc==0)).sum())
    assert int(df.has_rmc.sum())==rmc_count
    if len(df):
        assert np.all(np.diff(df.t_utc_ns.to_numpy(dtype=np.int64))>0)
    qc = dict(log_lines=sum(counts.values())+len(malformed)-len(recovered),sentence_counts=dict(counts),collector_syntax_sentence_counts=dict(collector_counts),valid_sentence_counts=dict(valid_counts),checksum_failures=len(errors),checksum_failure_sentences=errors,malformed_lines=malformed,invalid_physical_log_lines=len(errors)+len(malformed),checksum_count_definition="Lines beginning with a NMEA marker that fail checksum validation; continuation fragments are reported separately as malformed.",epochs=len(df),valid_epochs=int(df.valid.sum()),rmc_epoch_count=int(df.has_rmc.sum()),gga_without_rmc_epochs=int(((df.has_gga==1)&(df.has_rmc==0)).sum()),incomplete_epochs=int(df.flag_incomplete_epoch.sum()),time_mismatch_epochs=int(df.flag_time_mismatch.sum()),gap_before_epochs=int(df.flag_gap_before.sum()))
    qc["recovered_embedded_sentences"]=recovered
    qc["sentences_dropped_before_first_epoch"]=dropped_before_first_epoch[0]
    qc["valid_definition"]="position valid: lat/lon present, GGA fix_quality>=1 and RMC status A when an RMC exists; epochs whose RMC was corrupted keep valid=1 with speed/course from a recovered VTG (speed_source=VTG) and blank gps_utc"
    qc["independent_log_epoch_count"] = rmc_count+qc["gga_without_rmc_epochs"]
    qc["collector_crosscheck"] = crosscheck_gps(session,df,matches)
    qc["clock"] = gps_clock(df,session)
    return df,qc


def equal_value(a,b):
    if pd.isna(a) and pd.isna(b):
        return True
    if pd.isna(a) or pd.isna(b):
        return False
    try:
        return math.isclose(float(a),float(b),rel_tol=1e-9,abs_tol=0)
    except (ValueError,TypeError):
        return str(a)==str(b)


def crosscheck_gps(session,df,matches):
    collector = read_csv(session/"gps/gps.csv")
    mapping = {kind:{m[kind]:i for i,m in enumerate(matches) if kind in m} for kind in ("RMC","GGA")}
    fields = {"RMC":{"latitude_deg":"lat_deg","longitude_deg":"lon_deg","ground_speed_mps":"speed_mps","course_deg":"course_deg","gps_utc":"gps_utc"},"GGA":{"altitude_m":"alt_msl_m","fix_quality":"fix_quality","satellite_count":"sats_used","hdop":"hdop"}}
    mismatch,unmatched,seen = [],[],{kind:[] for kind in mapping}
    for i,row in collector.iterrows():
        kind = str(row.source_sentence)[-3:]
        if kind not in mapping:
            continue
        mono=int(row.host_monotonic_ns)
        if mono not in mapping[kind]:
            unmatched.append(dict(collector_row=int(i),kind=kind,host_monotonic_ns=mono))
            continue
        seen[kind].append(mono)
        target=df.iloc[mapping[kind][mono]]
        for c,out in fields[kind].items():
            if not equal_value(row[c],target[out]):
                mismatch.append(dict(collector_row=int(i),epoch_idx=int(target.epoch_idx),field=c,collector=None if pd.isna(row[c]) else row[c],refined=None if pd.isna(target[out]) else target[out]))
    reverse={kind:sorted(set(mapping[kind])-set(seen[kind])) for kind in mapping}
    duplicates={kind:len(seen[kind])-len(set(seen[kind])) for kind in mapping}
    result=dict(collector_rows=len(collector),mismatch_count=len(mismatch),mismatches=mismatch,unmatched_collector_count=len(unmatched),unmatched_collector=unmatched,unmatched_refined_counts={k:len(v) for k,v in reverse.items()},unmatched_refined_monotonic_ns=reverse,duplicate_match_counts=duplicates)
    assert not mismatch, f"GPS collector mismatch: {mismatch[:3]}"
    assert not unmatched and not any(reverse.values()) and not any(duplicates.values()), result
    return result


def gps_clock(df,session):
    if not len(df):
        return dict(leap_offset_epochs=0,leap_intervals=[],leap_transitions=[])
    mono=df.host_monotonic_ns.to_numpy(dtype=np.int64)
    wall=df.t_utc_ns.to_numpy(dtype=np.int64)
    offsets=wall-mono
    leap=df.flag_leap_offset.to_numpy(dtype=int)
    values=pd.to_numeric(df.host_minus_gps_s).to_numpy(dtype=float)
    starts=np.flatnonzero((leap==1)&(np.r_[0,leap[:-1]]==0))
    ends=np.flatnonzero((leap==1)&(np.r_[leap[1:],0]==0))
    transitions=[]
    for i in np.flatnonzero(np.diff(leap)!=0)+1:
        pair=[]
        for j in [i-1,i]:
            row=df.iloc[j]
            pair.append(dict(epoch_idx=j,t_utc_ns=int(row.t_utc_ns),t_kst=kst(row.t_utc_ns),gps_utc=row.gps_utc,gps_utc_ns=row.gps_utc_ns,flag_leap_offset=int(row.flag_leap_offset)))
        transitions.append(dict(before=pair[0],after=pair[1],raw_host_minus_gps_jump_s=((wall[i]-int(df.gps_utc_ns.iloc[i]))-(wall[i-1]-int(df.gps_utc_ns.iloc[i-1])))/1e9 if not pd.isna(df.gps_utc_ns.iloc[i]) and not pd.isna(df.gps_utc_ns.iloc[i-1]) else None))
    mask=np.isfinite(values)
    slope=np.polyfit((mono[mask]-mono[0])/1e9,values[mask],1)[0]*60000 if mask.sum()>1 else None
    events=read_events(session)
    start=next((e for e in events if e.get("kind")=="session_started"),None)
    workers=[e for e in events if e.get("kind") in ("worker_started","workers_started")]
    event_clock=[]
    for e in ([start] if start else [])+workers:
        if "host_wall_time_ns" in e and "host_monotonic_ns" in e:
            off=int(e["host_wall_time_ns"])-int(e["host_monotonic_ns"])
            event_clock.append(dict(kind=e["kind"],source=e.get("source"),t_kst=kst(e["host_wall_time_ns"]),wall_minus_monotonic_ns=off,delta_from_session_started_ms=(off-(int(start["host_wall_time_ns"])-int(start["host_monotonic_ns"])))/1e6 if start else None))
    corrected=df.gps_utc_corrected_ns
    has=corrected.notna().to_numpy()
    gps_minus_mono=(corrected[has].to_numpy(dtype=np.int64)-mono[has])
    mono_ppm=np.polyfit((mono[has]-mono[has][0])/1e9,(gps_minus_mono-gps_minus_mono[0])/1e9,1)[0]*1e6 if has.sum()>1 else None
    step=None
    first_worker=next((e for e in events if e.get("kind")=="worker_started"),None)
    if start and first_worker:
        step=dict(session_started_wall_kst=kst(start["host_wall_time_ns"]),first_worker_started_wall_kst=kst(first_worker["host_wall_time_ns"]),monotonic_elapsed_s=(int(first_worker["host_monotonic_ns"])-int(start["host_monotonic_ns"]))/1e9,wall_elapsed_s=(int(first_worker["host_wall_time_ns"])-int(start["host_wall_time_ns"]))/1e9)
        step["wall_minus_monotonic_change_s"]=step["wall_elapsed_s"]-step["monotonic_elapsed_s"]
    utc=df.gps_utc.dropna()
    dup=utc[utc.duplicated(keep=False)]
    return dict(leap_offset_epochs=int(sum(leap)),leap_intervals=[dict(first_epoch_idx=int(s),last_epoch_idx=int(e),epochs=int(e-s+1)) for s,e in zip(starts,ends)],single_initial_contiguous_leap_interval=bool(len(starts)==1 and starts[0]==0),leap_transitions=transitions,repeated_gps_utc_values=sorted(set(dup)),repeated_gps_utc_rows=int(len(dup)),corrected_pre_leap=distribution(values[leap==1]),corrected_post_leap=distribution(values[leap==0]),corrected_host_minus_gps_60s=bins60(wall,values),linear_trend_ms_per_min=slope,wall_minus_monotonic_change_ms=(int(offsets[-1])-int(offsets[0]))/1e6,wall_minus_monotonic_range_ms=(int(offsets.max())-int(offsets.min()))/1e6,wall_minus_monotonic_60s=[dict(**b,offset_reference_ns=int(offsets[0])) for b in bins60(wall,(offsets-offsets[0])/1e6)],wall_minus_monotonic_60s_unit="median is ms relative to offset_reference_ns",event_clock=event_clock,gps_minus_monotonic_trend_ppm=mono_ppm,gps_minus_monotonic_trend_definition="slope of (corrected GPS UTC - host_monotonic_ns); positive = host monotonic oscillator runs slow vs GPS",clock_step_before_recording=step,event_clock_interpretation="Differences are observed offsets; event clock-domain changes cannot be separated from wall steps using these files alone.")


def read_events(session):
    path=session/"events.jsonl"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()] if path.exists() else []


def ecef(lat,lon,height):
    lat,lon=np.radians(lat),np.radians(lon)
    a,e2=6378137.0,6.6943799901413165e-3
    n=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.column_stack(((n+height)*np.cos(lat)*np.cos(lon),(n+height)*np.cos(lat)*np.sin(lon),(n*(1-e2)+height)*np.sin(lat)))


def enu(df,origin=None):
    if origin is None:
        valid=df[(df.valid==1)&df.lat_deg.notna()&df.lon_deg.notna()]
        assert len(valid),"Run2 needs a valid origin fix"
        idx=valid.index[0]
        row=df.loc[idx]
        alt=row.alt_msl_m
        if pd.isna(alt):
            alt=df.alt_msl_m.dropna().iloc[0]
        geoid=row.geoid_sep_m
        if pd.isna(geoid):
            geoid=df.geoid_sep_m.dropna().iloc[0]
        origin=dict(lat_deg=float(row.lat_deg),lon_deg=float(row.lon_deg),alt_msl_m=float(alt),geoid_sep_m=float(geoid),height_ellipsoid_m=float(alt+geoid),epoch_idx=int(idx),t_utc_ns=int(row.t_utc_ns),datum="WGS84",altitude_method="ellipsoid height = GGA MSL altitude + geoid separation; missing row heights leave all ENU coordinates missing")
    for c in ("east_m","north_m","up_m"):
        df[c]=np.nan
    if len(df):
        mask=df[["lat_deg","lon_deg","alt_msl_m","geoid_sep_m"]].notna().all(axis=1)
        lat,lon=math.radians(origin["lat_deg"]),math.radians(origin["lon_deg"])
        r=np.array([[-math.sin(lon),math.cos(lon),0],[-math.sin(lat)*math.cos(lon),-math.sin(lat)*math.sin(lon),math.cos(lat)],[math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat)]])
        xyz=ecef(df.loc[mask,"lat_deg"].to_numpy(dtype=float),df.loc[mask,"lon_deg"].to_numpy(dtype=float),(df.loc[mask,"alt_msl_m"]+df.loc[mask,"geoid_sep_m"]).to_numpy(dtype=float))
        o=ecef(np.array([origin["lat_deg"]]),np.array([origin["lon_deg"]]),np.array([origin["height_ellipsoid_m"]]))[0]
        df.loc[mask,["east_m","north_m","up_m"]]=(xyz-o)@r.T
    return origin


def parse_bin(path):
    if not path.exists():
        return None,dict(available=False,status="NOT_VERIFIED")
    data=path.read_bytes()
    rows,current=[],{}
    frames,bad,skipped=Counter(),0,0
    pos=0
    def finish():
        if current:
            row={}
            for typ,words in current.items():
                names={0x51:["raw_acc_x","raw_acc_y","raw_acc_z"],0x52:["raw_gyro_x","raw_gyro_y","raw_gyro_z"],0x53:["raw_roll","raw_pitch","raw_yaw"],0x54:["raw_mag_x","raw_mag_y","raw_mag_z"]}[typ]
                row.update(zip(names,words[:3]))
            temp=current.get(0x51,current.get(0x54))
            row["temperature_c"]=temp[3]/100 if temp else None
            rows.append(row)
    while pos+11<=len(data):
        if data[pos]!=0x55:
            skipped+=1;pos+=1;continue
        frame=data[pos:pos+11]
        if sum(frame[:10])&255 != frame[10]:
            bad+=1;skipped+=1;pos+=1;continue
        typ=frame[1]
        frames[f"0x{typ:02x}"]+=1
        pos+=11
        if typ not in (0x51,0x52,0x53,0x54):
            continue
        if typ in current:
            finish();current={}
        current[typ]=struct.unpack("<4h",frame[2:10])
        if len(current)==4:
            finish();current={}
    finish()
    return rows,dict(available=True,bytes=len(data),samples=len(rows),valid_frames=dict(frames),checksum_failures=bad,discarded_bytes=skipped,trailing_bytes=len(data)-pos)


def compare_bin(df,path):
    rows,qc=parse_bin(path)
    if rows is None:
        return qc
    raw_mismatches=Counter()
    si_errors={}
    assert len(rows)==len(df), f"Binary sample count {len(rows)} != CSV {len(df)}: {path}"
    for c in [x for x in df if x.startswith("raw_") and not x.startswith("raw_q")]:
        for i,row in enumerate(rows):
            if not equal_value(df[c].iloc[i],row.get(c,np.nan)):
                raw_mismatches[c]+=1
    columns={}
    for axis in "xyz":
        columns[f"acc_{axis}_mps2"]=(f"raw_acc_{axis}",16/32768*9.80665)
        columns[f"gyro_{axis}_radps"]=(f"raw_gyro_{axis}",2000/32768*math.pi/180)
        columns[f"mag_{axis}"]=(f"raw_mag_{axis}",1)
    for c in ["roll","pitch","yaw"]:
        columns[c+"_rad"]=("raw_"+c,180/32768*math.pi/180)
    for c,(raw,scale) in columns.items():
        calc=np.array([row.get(raw,np.nan) for row in rows],dtype=float)*scale
        actual=pd.to_numeric(df[c]).to_numpy(dtype=float)
        si_errors[c]=float(np.nanmax(abs(actual-calc))) if len(calc) and np.isfinite(calc).any() else None
        assert np.array_equal(np.isnan(calc),np.isnan(actual)),c
    temp=np.array([row.get("temperature_c",np.nan) for row in rows],dtype=float)
    actual=pd.to_numeric(df.temperature_c).to_numpy(dtype=float)
    temp_mismatch=int(sum(not equal_value(a,b) for a,b in zip(actual,temp)))
    qc.update(raw_integer_mismatch_count=sum(raw_mismatches.values()),raw_integer_mismatches=dict(raw_mismatches),temperature_mismatch_count=temp_mismatch,temperature_max_abs_error=float(np.nanmax(abs(actual-temp))) if len(temp) else None,si_max_abs_error=si_errors)
    qc["matches_csv"]=bool(not raw_mismatches and temp_mismatch==0 and all(v is None or v<1e-12 for v in si_errors.values()))
    assert qc["matches_csv"],qc
    return qc


def reconstruct_time(df):
    """Sample time = lower envelope of host arrival vs sensor sequence (delays are one-sided)."""
    n=len(df)
    if not n:
        return df,dict(reconstructed=False,reason="empty session")
    mono=df.host_monotonic_ns.to_numpy(dtype=np.int64)
    wall=df.host_wall_time_ns.to_numpy(dtype=np.int64)
    seq=df.sensor_sequence.to_numpy(dtype=np.int64)
    x=(seq-seq[0]).astype(float)
    y=(mono-mono[0])/1e9
    dt=np.r_[np.nan,np.diff(mono)/1e9]
    shared=np.r_[False,np.diff(mono)==0] | np.r_[np.diff(mono)==0,False]
    # Partial flush samples at session stop arrive off-cadence: keep them out of the fit.
    packets=df.packets_in_sample.to_numpy(dtype=np.int64)
    use=(df.sample_valid.to_numpy(dtype=np.int64)==1)&(packets==Counter(packets.tolist()).most_common(1)[0][0])
    enough=use.sum()>=2
    ordinary=np.polyfit(x[use],y[use],1) if enough else np.array([0.,0.])
    residual=y-np.polyval(ordinary,x)
    block=x//100
    bx,by=[],[]
    for blk in np.unique(block[use]):
        ii=np.flatnonzero((block==blk)&use)
        j=ii[np.argmin(residual[ii])]
        bx.append(x[j]);by.append(y[j])
    period,intercept=np.polyfit(bx,by,1) if len(bx)>=2 else ordinary
    line=intercept+period*x
    envelope=np.zeros(n)
    slow=bool(use.sum()>=ENVELOPE_MIN_SAMPLES)
    if slow:
        # Slow sensor-clock rate wander: rolling minimum of the delay (60 s), then 120 s mean.
        delay=pd.Series(np.where(use,y-line,np.nan))
        envelope=delay.rolling(601,center=True,min_periods=1).min().rolling(1201,center=True,min_periods=1).mean().to_numpy()
    model=line+envelope
    model+=float(np.min((y-model)[use])) if use.any() else 0.0
    lower=np.array([np.percentile((y-line)[(block==blk)&use],5) for blk in np.unique(block[use])]) if use.any() else np.array([])
    lower_range=float(np.ptp(lower)) if len(lower) else 0
    period_deviation=abs(period/ordinary[0]-1) if enough and ordinary[0]>0 else np.inf
    # Fixed gate: stable cadence, envelope fit consistent with OLS, flat lower envelope.
    eligible=bool(use.sum()>=50 and 0.08<=period<=0.12 and lower_range<=0.04 and np.all(np.diff(seq)==1) and period_deviation<PERIOD_DEVIATION_MAX)
    fit=mono[0]+np.rint(model*1e9).astype(np.int64) if eligible else mono.copy()
    fit=np.minimum(fit,mono)  # a sample cannot be stamped after its own arrival
    offsets=wall-mono
    unique,ii=np.unique(mono,return_index=True)
    offset_ref=int(offsets[0])
    interp=np.interp((fit-unique[0])/1e9,(unique-unique[0])/1e9,(offsets[ii]-offset_ref).astype(float))
    utc=fit+offset_ref+np.rint(interp).astype(np.int64) if eligible else wall.copy()
    df.insert(0,"t_utc_ns",pd.array(utc,dtype="Int64"))
    df["host_delay_ms"]=(wall-utc)/1e6
    df["dt_host_s"]=dt
    df["flag_host_burst"]=pd.array(shared.astype(int),dtype="Int64")
    df["flag_gap_before"]=pd.array((dt>0.15).astype(int),dtype="Int64")
    df["flag_ts_reconstructed"]=pd.array(np.full(n,int(eligible)),dtype="Int64")
    minute=((mono-mono[0])//60_000_000_000)
    delay_ms=(wall-utc)/1e6
    qc=dict(reconstructed=eligible,reason=("block-minima line through the arrival lower envelope"+(" plus slow envelope (60 s rolling min, 120 s mean)" if slow else "")) if eligible else "Host retained: fewer than 50 fit samples or cadence/flat-envelope/sequence/period-vs-OLS gate failed",gate=dict(min_samples=50,period_ms_range=[80,120],lower_envelope_range_max_ms=40,sequence_step_required=1,period_vs_ols_max_relative=PERIOD_DEVIATION_MAX,slow_envelope_min_samples=ENVELOPE_MIN_SAMPLES),fit_rows=int(use.sum()),rows_excluded_from_fit=int((~use).sum()),slow_envelope_applied=bool(slow and eligible),estimated_period_ms=period*1000,ols_period_ms=ordinary[0]*1000,period_vs_ols_relative=period_deviation if np.isfinite(period_deviation) else None,ols_residual_ms=distribution(residual*1000),lower_envelope_p5_ms=lower*1000,lower_envelope_range_ms=lower_range*1000,slow_envelope_range_ms=float(np.ptp(envelope))*1000,host_delay_ms=distribution(delay_ms),host_delay_p5_per_minute_ms=[float(np.percentile(delay_ms[(minute==m)&use],5)) for m in np.unique(minute[use])] if use.any() else [],reconstructed_dt_s=distribution(np.diff(utc)/1e9),reconstructed_monotonic_dt_s=distribution(np.diff(fit)/1e9),fit_reference_sequence=int(seq[0]),host_dt_s=distribution(dt),same_timestamp_excess_rows=int(sum(np.diff(mono)==0)),host_burst_flag_rows=int(shared.sum()),host_dt_gt_0_3_s=int(sum(dt>0.3)),host_dt_gt_0_15_s=int(sum(dt>0.15)),sensor_sequence_steps=dict(Counter(map(int,np.diff(seq)))),mono_wall_interpolation="Interpolate integer wall-minus-mono offsets with relative monotonic seconds; clamp outside observed host range.")
    if eligible and slow:
        d=np.diff(utc[use])/1e9
        assert d.min()>0.0990 and d.max()<0.1000,(d.min(),d.max())
    return df,qc


def imu_session(session):
    source=read_csv(session/"imu/imu.csv")
    raw_cols=list(source.columns)
    dedup=source.drop_duplicates().reset_index(drop=True)
    qc=dict(input_rows=len(source),complete_duplicate_rows=len(source)-len(dedup),rows=len(dedup))
    int_cols=[c for c in dedup if c.endswith("_ns") or c.startswith("raw_") or c in ("sensor_sequence","packets_in_sample","checksum_valid","sample_valid")]
    integerize(dedup,int_cols)
    for c in dedup:
        if c not in int_cols and c!="packet_type":
            dedup[c]=pd.to_numeric(dedup[c])
    qc["bin_crosscheck"]=compare_bin(dedup,session/"imu/raw_000.bin")
    exclude=[c for c in dedup if c.startswith("quaternion_") and dedup[c].isna().all()]
    # Preserve every raw integer column, including absent raw_q* values.
    qc["excluded_all_nan_quaternion_columns"]=exclude
    redundant=[c for c in ("mag_x","mag_y","mag_z") if c in dedup and (dedup[c].fillna(0)==dedup["raw_"+c].fillna(0)).all() and dedup[c].isna().equals(dedup["raw_"+c].isna())]
    qc["excluded_redundant_mag_columns"]=redundant
    dedup=dedup.drop(columns=exclude+redundant+[c for c in dedup if c.startswith("veh_")])
    if not len(dedup):
        for c in IMU_PREFIX:
            if c not in dedup:
                dedup[c]=pd.Series(dtype="Int64" if c.endswith("_ns") or c.startswith("flag_") or c=="sensor_sequence" else float)
        qc["timing"]=dict(reconstructed=False,reason="empty session")
    else:
        assert np.all(np.diff(dedup.sensor_sequence.to_numpy(dtype=np.int64))>0)
        dedup,qc["timing"]=reconstruct_time(dedup)
        assert np.all(np.diff(dedup.t_utc_ns.to_numpy(dtype=np.int64))>0),f"Non-increasing IMU UTC: {session.name}"
    dedup=dedup.reindex(columns=IMU_PREFIX+[c for c in dedup if c not in IMU_PREFIX])
    assert len(dedup)==len(source)-qc["complete_duplicate_rows"]
    qc["original_columns"]=raw_cols
    return dedup,qc


def relation(actual,reference,mask=None):
    actual,reference=np.asarray(actual,dtype=float),np.asarray(reference,dtype=float)
    ok=np.isfinite(actual)&np.isfinite(reference)
    if mask is not None:
        ok &= mask
    a,b=actual[ok],reference[ok]
    if len(a)<3 or np.std(a)==0 or np.std(b)==0:
        return dict(n=len(a),corr=None,slope=None,intercept=None,rmse=None)
    slope,intercept=np.polyfit(b,a,1)
    return dict(n=len(a),corr=float(np.corrcoef(a,b)[0,1]),slope=slope,intercept=intercept,rmse=float(np.sqrt(np.mean((a-b)**2))),regression="actual = slope * reference + intercept; RMSE uses identity")


def angle_fit(horizontal,target,mask):
    ok=mask & np.isfinite(horizontal).all(axis=1)&np.isfinite(target).all(axis=1)
    h,t=horizontal[ok],target[ok]
    if len(h)<3:
        return np.nan
    dot=np.sum(h*t)
    cross=np.sum(h[:,0]*t[:,1]-h[:,1]*t[:,0])
    return math.atan2(cross,dot)


def rotation2(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


def angular_difference(a,b):
    return abs(math.degrees(math.atan2(math.sin(a-b),math.cos(a-b))))


def smooth_derivative(t,y):
    result=np.full(len(y),np.nan)
    usable=np.isfinite(y)
    starts=np.flatnonzero(usable & ~np.r_[False,usable[:-1]])
    ends=np.flatnonzero(usable & ~np.r_[usable[1:],False])+1
    for start,end in zip(starts,ends):
        cuts=np.r_[start,np.flatnonzero(np.diff(t[start:end])>1.5)+start+1,end]
        for a,b in zip(cuts[:-1],cuts[1:]):
            if b-a>=3:
                yy=y[a:b]
                if b-a>=5:
                    yy=savgol_filter(yy,5,2)
                result[a:b]=np.gradient(yy,t[a:b],edge_order=2)
    return result


def epoch_average(t,values,grid):
    out=np.full((len(grid),values.shape[1]),np.nan)
    for i,center in enumerate(grid):
        a,b=np.searchsorted(t,[center-0.5,center+0.5])
        if b>a:
            out[i]=np.mean(values[a:b],axis=0)
    return out


def mounting(imu,gps):
    base=int(imu.t_utc_ns.iloc[0])
    ti=(imu.t_utc_ns.to_numpy(dtype=np.int64)-base)/1e9
    tg=(gps.t_utc_ns.to_numpy(dtype=np.int64)-base)/1e9
    speed=gps.speed_mps.to_numpy(dtype=float, copy=True)
    valid=gps.valid.to_numpy(dtype=int)==1
    speed[~valid]=np.nan
    acc=imu[[f"acc_{a}_mps2" for a in "xyz"]].to_numpy(dtype=float)
    gyro=imu[[f"gyro_{a}_radps" for a in "xyz"]].to_numpy(dtype=float)
    stationary=(speed<0.3)&valid
    stops=[]
    starts=np.flatnonzero(stationary&~np.r_[False,stationary[:-1]])
    ends=np.flatnonzero(stationary&~np.r_[stationary[1:],False])
    station_mask=np.zeros(len(imu),dtype=bool)
    for a,b in zip(starts,ends):
        if tg[b]-tg[a]<3 or np.any(np.diff(tg[a:b+1])>1.5):
            continue
        mask=(ti>=tg[a])&(ti<=tg[b])&(np.linalg.norm(gyro,axis=1)<GYRO_STATIONARY_MAX_RADPS)
        if mask.sum()<20:
            continue
        station_mask|=mask
        stops.append(dict(gps_first_epoch=int(a),gps_last_epoch=int(b),duration_s=tg[b]-tg[a],imu_samples=int(mask.sum()),mean_acc_sensor_mps2=np.mean(acc[mask],axis=0),mean_gyro_sensor_radps=np.mean(gyro[mask],axis=0)))
    assert station_mask.sum()>0,"No stationary GPS/gyro intervals"
    # Unweighted component-wise median of per-stop means: one long stop (e.g. a sloped
    # parking spot before departure) must not dominate the level reference.
    stop_means=np.array([st["mean_acc_sensor_mps2"] for st in stops])
    gs=np.median(stop_means,axis=0)
    gs=gs/np.linalg.norm(gs)*np.median(np.linalg.norm(stop_means,axis=1))
    z=gs/np.linalg.norm(gs)
    x=np.array([1.,0.,0.])-z[0]*z
    x/=np.linalg.norm(x)
    y=np.cross(z,x)
    level=np.array([x,y,z])
    leveled_acc=acc@level.T
    leveled_gyro=gyro@level.T
    long=smooth_derivative(tg,speed)
    course=gps.course_deg.to_numpy(dtype=float)
    usable=np.isfinite(course)&(speed>3)&valid
    # Unwrap only consecutive usable spans to avoid interpolating stationary heading.
    unwrapped=np.full(len(course),np.nan)
    st=np.flatnonzero(usable&~np.r_[False,usable[:-1]])
    en=np.flatnonzero(usable&~np.r_[usable[1:],False])+1
    for a,b in zip(st,en):
        unwrapped[a:b]=np.unwrap(np.radians(course[a:b]))
    course_rate=-smooth_derivative(tg,unwrapped)
    moving=(speed>2)&valid&np.isfinite(long)
    candidates=[]
    for lag in np.round(np.arange(-3,3.001,0.1),1):
        avg=epoch_average(ti,np.column_stack((leveled_acc[:,:2],leveled_gyro[:,2])),tg-lag)
        target=np.column_stack((long,speed*avg[:,2]))
        psi=angle_fit(avg[:,:2],target,moving)
        if not np.isfinite(psi):
            continue
        vehicle=avg[:,:2]@rotation2(psi).T
        metric=relation(vehicle[:,0],long,moving)
        candidates.append((metric["corr"] if metric["corr"] is not None else -2,lag,psi,avg,target))
    assert candidates,"No dynamic samples for yaw estimation"
    _,lag,psi,avg,target=max(candidates,key=lambda v:v[0])
    r=np.eye(3);r[:2,:2]=rotation2(psi)
    r=r@level
    vehicle_acc=acc@r.T
    vehicle_gyro=gyro@r.T
    va=avg[:,:2]@rotation2(psi).T
    corr_long=relation(va[:,0],long,moving)
    corr_lat=relation(va[:,1],target[:,1],moving)
    corr_yaw=relation(avg[:,2],course_rate,(speed>3)&valid)
    target_course=np.column_stack((long,speed*course_rate))
    pc=angle_fit(avg[:,:2],np.column_stack((np.zeros(len(long)),target_course[:,1])),moving&np.isfinite(course_rate))
    segments=[]
    for k in range(int((tg[-1]-tg[0])//240)+1):
        m=moving&(tg>=tg[0]+240*k)&(tg<tg[0]+240*(k+1))
        if m.sum()>=30:
            seg=angle_fit(avg[:,:2],target,m)
            segments.append(dict(t_start_kst=kst(base+int((tg[0]+240*k)*1e9)),n=int(m.sum()),psi_minus_joint_deg=math.degrees(math.atan2(math.sin(seg-psi),math.cos(seg-psi)))))
    peak=max(v[0] for v in candidates)
    lag_ok=[v[1] for v in candidates if v[0]>=0.99*peak]
    early=moving&(tg<np.median(tg))
    late=moving&(tg>=np.median(tg))
    psi1,psi2=angle_fit(avg[:,:2],target,early),angle_fit(avg[:,:2],target,late)
    only_long=np.column_stack((long,np.zeros(len(long))))
    only_lat=np.column_stack((np.zeros(len(long)),target[:,1]))
    # Each single-axis objective keeps only that target component in dot/cross.
    pl=angle_fit(avg[:,:2],only_long,moving)
    pa=angle_fit(avg[:,:2],only_lat,moving)
    block=((tg-tg[0])//60).astype(int)
    blocks=np.unique(block[moving])
    rng=np.random.default_rng(SEED)
    boots=[]
    for _ in range(300):
        chosen=rng.choice(blocks,size=len(blocks),replace=True)
        indices=np.concatenate([np.flatnonzero((block==b)&moving) for b in chosen])
        hp,tp=avg[indices,:2],target[indices]
        angle=angle_fit(hp,tp,np.ones(len(indices),dtype=bool))
        if np.isfinite(angle):
            boots.append(math.degrees(math.atan2(math.sin(angle-psi),math.cos(angle-psi))))
    half_difference=angular_difference(psi1,psi2) if np.isfinite(psi1+psi2) else np.nan
    axis_difference=angular_difference(pl,pa) if np.isfinite(pl+pa) else np.nan
    bootstrap_std=float(np.std(boots,ddof=1)) if len(boots)>1 else np.nan
    uncertainty=max(half_difference,axis_difference,bootstrap_std) if np.all(np.isfinite([half_difference,axis_difference,bootstrap_std])) else np.nan
    ok=(corr_long["corr"] is not None and corr_yaw["corr"] is not None and corr_long["corr"]>=TRUST_CORR_LONG and corr_yaw["corr"]>=TRUST_CORR_YAW and np.isfinite(uncertainty) and uncertainty<=TRUST_YAW_UNCERTAINTY_DEG)
    trust="ok" if ok else ("degraded" if corr_long["corr"] is not None and corr_yaw["corr"] is not None and corr_long["corr"]>0 and corr_yaw["corr"]>0 else "untrusted")
    roll=math.atan2(r[2,1],r[2,2]);pitch=math.asin(-r[2,0]);yaw=math.atan2(r[1,0],r[0,0])
    gravity_roll=math.atan2(gs[1],gs[2]);gravity_pitch=math.atan2(-gs[0],math.hypot(gs[1],gs[2]))
    scatter=[]
    for stop in stops:
        g=np.asarray(stop["mean_acc_sensor_mps2"])
        stop["gravity_direction_deviation_deg"]=math.degrees(math.acos(np.clip(g@z/np.linalg.norm(g),-1,1)))
        scatter.append(stop["gravity_direction_deviation_deg"])
    static_mean=np.mean(vehicle_acc[station_mask],axis=0)
    gnorm=float(np.linalg.norm(gs))
    for stop in stops:
        vm=r@np.asarray(stop["mean_acc_sensor_mps2"])
        stop["mean_acc_vehicle_mps2"]=vm
        stop["implied_pitch_deg"]=math.degrees(math.asin(np.clip(vm[0]/gnorm,-1,1)))
        stop["implied_roll_deg"]=math.degrees(math.asin(np.clip(vm[1]/gnorm,-1,1)))
        stop["outlier"]=bool(stop["gravity_direction_deviation_deg"]>=OUTLIER_STOP_DEG)
    stop_vehicle=np.array([st["mean_acc_vehicle_mps2"] for st in stops])
    stop_tilt=np.array([[st["implied_pitch_deg"],st["implied_roll_deg"]] for st in stops])
    robust_sd=1.4826*np.median(abs(stop_tilt-np.median(stop_tilt,axis=0)),axis=0)
    level_sd=float(max(robust_sd))
    long_offset=float(np.nanmean(va[moving,0])-np.nanmean(long[moving]))
    turning=abs(vehicle_gyro[:,2])>0.1
    gyro_axis=dict(samples=int(turning.sum()),gyro_x_over_z=float(np.polyfit(vehicle_gyro[turning,2],vehicle_gyro[turning,0],1)[0]),gyro_y_over_z=float(np.polyfit(vehicle_gyro[turning,2],vehicle_gyro[turning,1],1)[0]))
    gyro_axis["tilt_about_y_deg"]=math.degrees(math.atan(gyro_axis["gyro_x_over_z"]))
    gyro_axis["tilt_about_x_deg"]=math.degrees(math.atan(gyro_axis["gyro_y_over_z"]))
    gyro_axis["note"]="Regression over 10 Hz samples with |veh_gyro_z|>0.1 rad/s; a nonzero ratio means the measured turn axis is tilted from vehicle z (levelling error and/or body roll/road camber in turns)."
    trust_yaw=trust
    trust_level="ok" if level_sd<=TRUST_LEVEL_SD_DEG and abs(long_offset)<TRUST_LONG_OFFSET_MPS2 else "degraded"
    order=["ok","degraded","untrusted"]
    trust=max(trust_yaw,trust_level,key=order.index)
    accelerating=moving&(long>0.8)
    decelerating=moving&(long< -0.8)
    accel_mean=float(np.nanmean(va[accelerating,0]))
    decel_mean=float(np.nanmean(va[decelerating,0]))
    acceptance=dict(stationary_mean_vehicle_mps2=static_mean,per_stop_median_abs_horizontal_mps2=np.median(abs(stop_vehicle[:,:2]),axis=0),non_outlier_stops=int(sum(not st["outlier"] for st in stops)),stops=len(stops),moving_long_offset_mps2=long_offset,accelerating_epoch_count=int(accelerating.sum()),decelerating_epoch_count=int(decelerating.sum()),accelerating_mean_vehicle_x_mps2=accel_mean,decelerating_mean_vehicle_x_mps2=decel_mean,course_yaw_positive=corr_yaw["corr"] is not None and corr_yaw["corr"]>0)
    q=dict(R_sensor_to_vehicle=r,determinant=np.linalg.det(r),orthogonality_error=float(np.max(abs(r@r.T-np.eye(3)))),roll_deg=math.degrees(roll),pitch_deg=math.degrees(pitch),yaw_deg=math.degrees(yaw),euler_convention="R = Rz(yaw) @ Ry(pitch) @ Rx(roll), active column-vector sensor-to-vehicle rotation; +x forward +y left +z up",method="Level from the unweighted component-wise median of per-stop mean specific force; unit-scale 2D least-squares rotation to [GPS dv/dt, speed * leveled gyro z]. Joint lag scan -3..3s at 0.1s maximizes long acceleration correlation after refitting yaw. GPS derivative: central difference after 5-epoch quadratic Savitzky-Golay smoothing. IMU means within +/-0.5s at shifted GPS host arrivals.",stationary_intervals=stops,stationary_imu_samples=int(station_mask.sum()),mean_gravity_sensor_mps2=gs,gravity_norm_mps2=np.linalg.norm(gs),tilt_deg=math.degrees(math.acos(z[2])),gravity_direction_scatter_deg=distribution(scatter),gyro_bias_radps=np.mean(gyro[station_mask],axis=0),gyro_bias_applied=False,gps_speed_lag_s=float(lag),psi_without_lag_deg=math.degrees(next(v[2] for v in candidates if v[1]==0)),corr_long_without_lag=float(next(v[0] for v in candidates if v[1]==0)),gps_speed_lag_sign="positive = GPS speed is later than IMU; IMU evaluated at GPS RMC host arrival minus lag",gps_time_basis="gps.t_utc_ns = RMC host wall arrival, not gps_utc",lag_scan=[dict(lag_s=float(v[1]),corr_long=float(v[0]),psi_deg=math.degrees(v[2])) for v in candidates],psi_leveling_deg=math.degrees(psi),longitudinal=corr_long,lateral=corr_lat,yaw_vs_negative_course_rate=corr_yaw,psi_uncertainty_deg=uncertainty,psi_uncertainty_method="maximum of first/second half difference, longitudinal-only/lateral-only difference, and 60s block bootstrap standard deviation",psi_first_half_deg=math.degrees(psi1),psi_second_half_deg=math.degrees(psi2),psi_half_difference_deg=half_difference,psi_long_only_deg=math.degrees(pl),psi_lat_only_deg=math.degrees(pa),psi_axis_difference_deg=axis_difference,psi_block_bootstrap_std_deg=bootstrap_std,bootstrap_seed=SEED,bootstrap_replicates=300,bootstrap_blocks=len(blocks),gravity_roll_pitch_deg=[math.degrees(gravity_roll),math.degrees(gravity_pitch)],builtin_stationary_roll_pitch_deg=np.degrees(imu.loc[station_mask,["roll_rad","pitch_rad"]].mean().to_numpy(dtype=float)),builtin_minus_gravity_roll_pitch_deg=np.degrees(imu.loc[station_mask,["roll_rad","pitch_rad"]].mean().to_numpy(dtype=float)-[gravity_roll,gravity_pitch]),trust=trust,trust_yaw=trust_yaw,trust_level=trust_level,leveling_uncertainty_deg=level_sd,leveling_uncertainty_definition="max over pitch/roll of 1.4826*MAD of per-stop implied tilt; road grade at stops is included",per_stop_tilt_sd_deg=dict(pitch_robust=float(robust_sd[0]),roll_robust=float(robust_sd[1]),pitch_plain=float(np.std(stop_tilt[:,0],ddof=1)),roll_plain=float(np.std(stop_tilt[:,1],ddof=1))),veh_acc_x_offset_mps2=long_offset,veh_acc_x_offset_definition="mean(veh_acc_x) - mean(GPS dv/dt) over moving epochs; includes mean road grade of the route",gyro_turn_axis=gyro_axis,gps_speed_lag_range_s=[float(min(lag_ok)),float(max(lag_ok))],gps_speed_lag_range_definition="lags whose longitudinal correlation is >= 99% of the peak (flat peak)",psi_lat_only_course_rate_deg=math.degrees(pc),psi_4min_segments=segments,psi_4min_segments_note="Segments with little excitation (first segment: parking-lot manoeuvres, 26 epochs above 5 m/s) are ill-conditioned; their deviation is not evidence of a mount change.",trust_level_note="trust_level uses the robust (MAD) spread of per-stop tilt because stops on sloped ground (departure spot about -4.5 deg pitch, one road stop about -2.1 deg) are road grade, not mount error; the plain sd is reported in per_stop_tilt_sd_deg.",road_grade_note="veh_acc_x/y contain the gravity component of road slope (e.g. about -0.78 m/s^2 in veh_acc_x during the first 84 s standstill): do not use the first standstill as a zero reference.",trust_criteria=dict(corr_long_min=TRUST_CORR_LONG,corr_yaw_min=TRUST_CORR_YAW,psi_uncertainty_max_deg=TRUST_YAW_UNCERTAINTY_DEG,leveling_sd_max_deg=TRUST_LEVEL_SD_DEG,long_offset_max_mps2=TRUST_LONG_OFFSET_MPS2),trust_reason=f"corr_long={corr_long['corr']}; corr_yaw={corr_yaw['corr']}; psi_uncertainty_deg={uncertainty}; all thresholds must pass for ok; positive correlations required for degraded",acceptance=acceptance,limitations="Stationary road slope and sensor bias are inseparable from mounting tilt; no gravity or gyro bias is removed. Yaw and lag use the same drive, without held-out validation.")
    assert abs(np.linalg.det(r)-1)<1e-10 and q["orthogonality_error"]<1e-10
    # Not circular: R is defined by the median stop, the check is on every stop.
    assert np.all(acceptance["per_stop_median_abs_horizontal_mps2"]<0.15) and 9.6<static_mean[2]<10.0,acceptance
    assert acceptance["non_outlier_stops"]*2>len(stops),acceptance
    assert abs(long_offset)<TRUST_LONG_OFFSET_MPS2,acceptance
    assert accel_mean>0 and decel_mean<0,acceptance
    assert acceptance["course_yaw_positive"],acceptance
    for i,axis in enumerate("xyz"):
        imu[f"veh_acc_{axis}_mps2"]=vehicle_acc[:,i]
    for i,axis in enumerate("xyz"):
        imu[f"veh_gyro_{axis}_radps"]=vehicle_gyro[:,i]
    imu["flag_mounting_untrusted"]=pd.array(np.full(len(imu),int(trust=="untrusted")),dtype="Int64")
    imu["flag_mounting_degraded"]=pd.array(np.full(len(imu),int(trust=="degraded")),dtype="Int64")
    plot_data=dict(stops=[(st["gps_first_epoch"],st["gps_last_epoch"]) for st in stops],t=tg,va=va,long=long,lat=target[:,1],yaw=avg[:,2],course_rate=course_rate,moving=moving,yaw_mask=(speed>3)&valid)
    return q,plot_data


def fragment_summary(session,gps,imu,gq,iq):
    valid=gps[gps.valid==1]
    speeds=valid.speed_kmh.dropna().to_numpy(dtype=float)
    position=valid[["east_m","north_m","up_m"]].to_numpy(dtype=float)
    if len(position)>1:
        step=np.linalg.norm(np.diff(position,axis=0),axis=1)
        consecutive=np.diff(valid.epoch_idx.to_numpy(dtype=int))==1
        distance=float(np.nansum(step[consecutive]))
    else:
        distance=0.0
    stamps=[]
    for d in (gps,imu):
        if len(d):
            stamps.extend([int(d.t_utc_ns.iloc[0]),int(d.t_utc_ns.iloc[-1])])
    events=read_events(session)
    note=Counter(e.get("kind","unknown") for e in events if e.get("level","").lower() in ("warning","warn","error"))
    return dict(session_id=session.name,incomplete=int((session/"INCOMPLETE").exists()),gps_epochs=len(gps),gps_valid_epochs=len(valid),gps_t_start_kst=kst(gps.t_utc_ns.iloc[0]) if len(gps) else "",gps_t_end_kst=kst(gps.t_utc_ns.iloc[-1]) if len(gps) else "",duration_s=(max(stamps)-min(stamps))/1e9 if stamps else 0,speed_max_kmh=float(max(speeds)) if len(speeds) else None,speed_mean_kmh=float(np.mean(speeds)) if len(speeds) else None,moving=int(len(speeds)>0 and max(speeds)>3),moved_distance_m=distance,leap_offset_epochs=int(gps.flag_leap_offset.sum()),host_minus_gps_median_s=float(gps.host_minus_gps_s.median()) if gps.host_minus_gps_s.notna().any() else None,imu_rows=len(imu),imu_t_start_kst=kst(imu.t_utc_ns.iloc[0]) if len(imu) else "",imu_t_end_kst=kst(imu.t_utc_ns.iloc[-1]) if len(imu) else "",has_raw_bin=int((session/"imu/raw_000.bin").exists()),bin_matches_csv=int(iq["bin_crosscheck"]["matches_csv"]) if "matches_csv" in iq["bin_crosscheck"] else None,note="; ".join(f"{k}:{v}" for k,v in sorted(note.items())))


def plot_time(ns):
    return [datetime.fromtimestamp(int(t)//10**9,KST)+timedelta(microseconds=(int(t)%10**9)//1000) for t in ns]


def saveplot(fig,name,time_axes=()):
    for ax in time_axes:
        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S",tz=KST))
        ax.set_xlabel("Time (KST)")
    fig.tight_layout()
    fig.savefig(QC/name,dpi=130)
    plt.close(fig)


def plots(gps,imu,fragments,data):
    times=plot_time(gps.t_utc_ns)
    fig,ax=plt.subplots(figsize=(8,7))
    usable=gps.valid==1
    scatter=ax.scatter(gps.loc[usable,"east_m"],gps.loc[usable,"north_m"],c=gps.loc[usable,"speed_kmh"],s=7,cmap="turbo")
    points=gps.loc[usable&gps.east_m.notna()]
    for idx,label,color in [(points.index[0],"Start","green"),(points.index[-1],"End","black")]:
        ax.scatter(gps.loc[idx,"east_m"],gps.loc[idx,"north_m"],marker="x",s=70,c=color,label=label)
    fig.colorbar(scatter,ax=ax,label="Speed (km/h)");ax.legend();ax.set_aspect("equal");ax.set(xlabel="East (m)",ylabel="North (m)",title="Run2 WGS84 ENU route")
    saveplot(fig,"gps_route_enu.png")
    fig,ax=plt.subplots(figsize=(12,4));ax.plot(times,gps.speed_kmh,label="RMC")
    leap=gps.flag_leap_offset.to_numpy(dtype=int)
    for a in np.flatnonzero((leap==1)&(np.r_[0,leap[:-1]]==0)):
        b=a
        while b+1<len(leap) and leap[b+1]:b+=1
        ax.axvspan(times[a],times[b],color="orange",alpha=.2)
    ax.set(ylabel="Speed (km/h)",title="Run2 speed; orange = GPS UTC offset period");saveplot(fig,"gps_speed.png",[ax])
    fig,axes=plt.subplots(2,1,figsize=(12,7),sharex=True)
    original=[(int(w)-int(u))/1e9 if not pd.isna(u) else np.nan for w,u in zip(gps.t_utc_ns,gps.gps_utc_ns)]
    axes[0].plot(times,original);axes[0].set(ylabel="Host - raw GPS (s)",title="GPS clock comparison")
    axes[1].plot(times,gps.host_minus_gps_s);axes[1].set(ylabel="Host - corrected GPS (s)");saveplot(fig,"gps_host_minus_gps.png",axes)
    fig,axes=plt.subplots(2,2,figsize=(13,8))
    for i,(actual,reference,title) in enumerate([(data["va"][:,0],data["long"],"Longitudinal acceleration"),(data["va"][:,1],data["lat"],"Lateral acceleration")]):
        axes[i,0].plot(times,actual,label="IMU mean");axes[i,0].plot(times,reference,label="GPS reference",alpha=.8);axes[i,0].legend();axes[i,0].set(ylabel="Acceleration (m/s^2)",title=title+" (grey = standstill)")
        for a,b in data["stops"]:
            axes[i,0].axvspan(times[a],times[b],color="gray",alpha=.2)
        mask=data["moving"]&np.isfinite(actual)&np.isfinite(reference)
        axes[i,1].scatter(reference[mask],actual[mask],s=7,alpha=.5);axes[i,1].set(xlabel="Reference (m/s^2)",ylabel="IMU (m/s^2)")
    saveplot(fig,"imu_acc_vs_gps.png",axes[:,0])
    fig,axes=plt.subplots(1,2,figsize=(13,4));axes[0].plot(times,data["yaw"],label="IMU gyro z");axes[0].plot(times,data["course_rate"],label="Negative course rate");axes[0].legend();axes[0].set(ylabel="Yaw rate (rad/s)")
    mask=data["yaw_mask"]&np.isfinite(data["course_rate"])&np.isfinite(data["yaw"])
    axes[1].scatter(data["course_rate"][mask],data["yaw"][mask],s=7);axes[1].set(xlabel="Negative course rate (rad/s)",ylabel="IMU yaw rate (rad/s)");saveplot(fig,"imu_yaw_vs_course.png",[axes[0]])
    fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True);it=plot_time(imu.host_wall_time_ns)
    axes[0].plot(it,imu.dt_host_s,linewidth=.5);axes[0].set(ylabel="Host dt (s)",title="IMU receive timing")
    axes[1].plot(it,imu.host_delay_ms,linewidth=.5);axes[1].set(ylabel="Host delay (ms)")
    axes[2].plot(it,imu.host_delay_ms,linewidth=.5);axes[2].set(ylabel="Host delay, zoom (ms)",ylim=(0,25))
    for i in np.flatnonzero(imu.flag_gap_before.to_numpy(dtype=int)):
        for ax in axes:ax.axvline(it[i],color="red",alpha=.2,linewidth=.6)
    saveplot(fig,"imu_timing.png",axes)
    fig,ax=plt.subplots(figsize=(13,5))
    for sid,df in fragments.groupby("session_id",sort=True):
        if len(df):
            t=plot_time(df.t_utc_ns);ax.plot(t,df.speed_kmh,linewidth=1,marker="o",markersize=3);ax.axvline(t[0],color="gray",alpha=.3,linewidth=.7)
            ax.text(t[0],56,sid[-6:],rotation=90,fontsize=6,va="top")
    ax.set(ylabel="Speed (km/h)",title="Run1 fragment speeds; lines mark session boundaries");saveplot(fig,"fragments_speed.png",[ax])


def self_check():
    iso,ns=utc_rmc("010126","073200.00")
    assert iso=="2026-01-01T07:32:00.000Z" and isinstance(ns,int)
    assert parse_nmea("$GPRMC,073200.00,A,3730.00000,N,12700.00000,E,0.062,,010126,,,D*70")[2]
    assert not parse_nmea("$GPRMC,073200.00,A,3730.00000,N,12700.00000,E,0.062,,010126,,,D*00")[2]
    x=ecef(np.array([0.]),np.array([0.]),np.array([0.]))
    assert np.allclose(x,[[6378137,0,0]])
    assert angular_difference(math.radians(179),math.radians(-179))<2.000001

    # Exercise arrival-order grouping and absent RMC/empty fragments independently.
    with tempfile.TemporaryDirectory(dir=ROOT/"_work/3_pi_gps_imu_camera",prefix="wp1_selfcheck_") as temp:
        session=Path(temp)
        (session/"gps").mkdir()
        (session/"imu").mkdir()
        def sentence(body):
            check=0
            for ch in body:
                check ^= ord(ch)
            return f"$"+body+f"*{check:02X}"
        rows,lines=[],[]
        base_mono,base_wall=100_000_000_000,1767252718057577721
        rmc="GPRMC,{time},A,3730.00000,N,12700.00000,E,0.062,,010126,,,D"
        gga="GPGGA,{time},3730.00000,N,12700.00000,E,2,06,2.63,33.7,M,17.8,M,,0000"
        for epoch,time in enumerate(["073200.00","073200.00","073201.00","073202.00"]):
            mono,wall=base_mono+epoch*10**9,base_wall+epoch*10**9
            if epoch in (0,1):
                raw=sentence(rmc.format(time=time))
                rows.append(dict(host_monotonic_ns=mono,source_sentence="GPRMC",latitude_deg=37.5,longitude_deg=127.0,ground_speed_mps=0.031895528,course_deg="",gps_utc="2026-01-01T07:32:00.000Z"))
            elif epoch==2:
                raw=sentence("GPVTG,,T,,M,0.062,N,0.114,K,D")
            else:
                raw=sentence(rmc.format(time=time))
                raw=raw[:-2]+("FF" if raw[-2:]!="FF" else "00")
            lines.append(f"{mono} {wall} {raw}")
            mono+=1_000_000;wall+=1_000_000
            lines.append(f"{mono} {wall} {sentence(gga.format(time=time))}")
            rows.append(dict(host_monotonic_ns=mono,source_sentence="GPGGA",altitude_m=33.7,fix_quality=2,satellite_count=6,hdop=2.63))
        (session/"gps/raw_nmea.log").write_text("\n".join(lines)+"\n",encoding="utf-8")
        fields="host_monotonic_ns source_sentence latitude_deg longitude_deg ground_speed_mps course_deg gps_utc altitude_m fix_quality satellite_count hdop".split()
        with (session/"gps/gps.csv").open("w",newline="",encoding="utf-8") as f:
            writer=csv.DictWriter(f,fieldnames=fields,lineterminator="\n")
            writer.writeheader();writer.writerows(rows)
        test,q=gps_epochs(session)
        assert len(test)==4 and test.gps_utc.iloc[0]==test.gps_utc.iloc[1]
        assert test.t_source.tolist()==["RMC","RMC","VTG","GGA"]
        assert test.has_vtg.iloc[2]==1 and test.flag_incomplete_epoch.tolist()==[0,0,1,1]
        assert q["checksum_failures"]==1 and q["collector_crosscheck"]["mismatch_count"]==0
        (session/"imu/imu.csv").write_bytes(b"")
        empty,_=imu_session(session)
        assert len(empty)==0
        (session/"imu/imu.csv").write_text("host_monotonic_ns,host_wall_time_ns,sensor_sequence\n",encoding="utf-8")
        empty,_=imu_session(session)
        assert len(empty)==0


def main():
    self_check()
    QC.mkdir(parents=True,exist_ok=True)
    gps,gq=gps_epochs(RUN2)
    origin=enu(gps)
    assert np.allclose(gps.loc[origin["epoch_idx"],["east_m","north_m"]].to_numpy(dtype=float),0,atol=1e-8)
    imu,iq=imu_session(RUN2)
    mount,data=mounting(imu,gps)
    stats=dict(run2=dict(gps=gq,imu=iq,mounting=mount),fragments={},acceptance_status="STARTED")
    gps_parts,imu_parts,summaries=[],[],[]
    sessions=sorted((ROOT/"00_raw/pi_run1_fragments").glob("session_*"))
    assert len(sessions)==28,f"Expected 28 fragment sessions, found {len(sessions)}"
    for session in sessions:
        fg,fgq=gps_epochs(session);enu(fg,origin)
        fi,fiq=imu_session(session)
        summaries.append(fragment_summary(session,fg,fi,fgq,fiq))
        stats["fragments"][session.name]=dict(gps=fgq,imu=fiq)
        fg.insert(1,"session_id",session.name);fi.insert(1,"session_id",session.name)
        gps_parts.append(fg);imu_parts.append(fi)
    fgs=pd.concat(gps_parts,ignore_index=True)
    fis=pd.concat(imu_parts,ignore_index=True)
    integerize(fgs,INT_GPS)
    integerize(fis,[c for c in fis if c.endswith("_ns") or c.startswith("raw_") or c.startswith("flag_") or c in ("sensor_sequence","packets_in_sample","checksum_valid","sample_valid")])
    for df,key in [(fgs,"gps"),(fis,"imu")]:
        for sid,part in df.groupby("session_id"):
            assert np.all(np.diff(part.t_utc_ns.to_numpy(dtype=np.int64))>0),sid
            assert len(part)==stats["fragments"][sid][key]["epochs" if key=="gps" else "rows"]
    for df,name in [(gps,"01_refined/run2/gps.csv"),(imu,"01_refined/run2/imu.csv"),(fgs,"01_refined/run1/gps_fragments.csv"),(fis,"01_refined/run1/imu_fragments.csv")]:
        write_csv(df,ROOT/name)
    write_json(origin,ROOT/"01_refined/run2/gps_origin.json")
    write_json(mount,ROOT/"01_refined/run2/imu_mounting.json")
    summary=pd.DataFrame(summaries)
    integerize(summary,["incomplete","gps_epochs","gps_valid_epochs","moving","leap_offset_epochs","imu_rows","has_raw_bin","bin_matches_csv"])
    summary.to_csv(QC/"fragments_summary.csv",index=False,encoding="utf-8",lineterminator="\n",na_rep="")
    stats["imu_timing_notes"]=dict(t_utc_ns_definition="estimated host arrival time of the last byte of the 44-byte sample (lower envelope of arrivals); not corrected for transport",imu_baud=IMU_BAUD,imu_baud_source="session_summary.json workers.imu.baud / daq.yaml",serial_transfer_time_44_bytes_ms=44*10/IMU_BAUD*1000,note="The measurement instant precedes t_utc_ns by at least the serial transfer time (plus unknown sensor-internal latency); nothing is subtracted.",fragment_fit_table=[dict(session_id=k,rows=v["imu"]["rows"],reconstructed=v["imu"]["timing"].get("reconstructed"),fit_period_ms=v["imu"]["timing"].get("estimated_period_ms"),ols_period_ms=v["imu"]["timing"].get("ols_period_ms"),max_host_delay_ms=(v["imu"]["timing"].get("host_delay_ms") or {}).get("max")) for k,v in stats["fragments"].items()])
    plots(gps,imu,fgs,data)
    stats["acceptance_status"]="PASS"
    stats["output_rows"]=dict(run2_gps=len(gps),run2_imu=len(imu),run1_gps_fragments=len(fgs),run1_imu_fragments=len(fis),fragments_summary=len(summary))
    stats["metadata_clock"]=json.loads((RUN2/"metadata.json").read_text(encoding="utf-8")).get("clock")
    stats["schema_notes"]=["GPS UTC/date never carried across bursts; missing RMC UTC remains blank.","GPS checksum-invalid lines counted and preserved only in stats.","ENU missing altitude/geoid yields missing ENU; path distance sums consecutive valid 3D ENU steps only.","IMU mag engineering columns omitted only where identical to preserved raw_mag columns.","raw_q* preserved as nullable integers; all-empty quaternion engineering columns omitted.","IMU mounting trust thresholds fixed in code; failed quality gates remain flagged even when physical sign assertions pass."]
    write_json(stats,QC/"stats_gps_imu.json")
    print(json.dumps(clean_json(dict(rows=stats["output_rows"],mounting_trust=mount["trust"],lag_s=mount["gps_speed_lag_s"],corr_long=mount["longitudinal"]["corr"],corr_yaw=mount["yaw_vs_negative_course_rate"]["corr"],psi_uncertainty_deg=mount["psi_uncertainty_deg"],acceptance="PASS")),ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
