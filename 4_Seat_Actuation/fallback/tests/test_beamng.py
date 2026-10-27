"""BeamNG row/observer checks with fake sockets only; raw recordings stay local."""
from __future__ import annotations

import contextlib
import csv
from dataclasses import FrozenInstanceError
import hashlib
import io
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))
from fallback.beamng import (VehicleValidator, VehicleFrame, BridgeReceiver, read_bridge_packet,
    read_vehicle_csv, MAX_DATAGRAM_BYTES, BRIDGE_HOST, BRIDGE_PORT, CLOCK_ID)
from fallback import beamng_bridge as bridge

LOCAL_COLLECTOR = Path(os.environ.get("BEAMNG_COLLECTOR_ROOT",
    r"C:\Users\SO\Desktop\BeamNG.simulation\BeamNG_Collector_v0_3\BeamNG_Collector_v0_3"))
LOCAL_DRIVE = LOCAL_COLLECTOR/"data/sessions/2026-10-25/session_20261025_061416_car_A_free_cbfe254c/vehicle/vehicle.csv"
LOCAL_STOP = LOCAL_COLLECTOR/"data/sessions/2026-10-25/session_20261025_060803_car_A_free_8623d582/vehicle/vehicle.csv"


def row(seq=1, *, stream="stream-A", time_ns=None, sim=None, **changes):
    value = dict(host_monotonic_ns=1_000_000_000+seq*20_000_000 if time_ns is None else time_ns,
        host_wall_time_ns=1_792_908_000_000_000_000, sim_time_s=seq*.02 if sim is None else sim,
        stream_id=stream, vehicle_id=24193, sequence_id=seq, source_kind="beamng",
        linear_acc_y_mps2=2., linear_acc_x_mps2=.2, speed_mps=12., steering_angle_deg=30.,
        throttle_input=.2, brake_input=0., derivative_valid=1, derivative_reason="ok",
        config_match=1, sample_dt_s=.02, missing_packets_before=0)
    value.update(changes)
    return value


def provenance():
    return dict(collector_root="offline collector fixture", session_id="offline-session-1",
                config_sha256="a"*64, source_sha256={"collect.py": "b"*64})


class FakeSocket:
    def __init__(self, *args):
        self.args, self.options, self.sent, self.incoming = args, [], [], []
        self.bound, self.blocking, self.closed = None, None, False
        self.send_error = None

    def setsockopt(self, *args):
        self.options.append(args)

    def setblocking(self, value):
        self.blocking = value

    def bind(self, address):
        self.bound = address

    def sendto(self, payload, address):
        if self.send_error:
            raise self.send_error
        self.sent.append((payload, address))
        return len(payload)

    def recvfrom(self, limit):
        self.last_receive_limit = limit
        if not self.incoming:
            raise BlockingIOError()
        return self.incoming.pop(0)

    def close(self):
        self.closed = True


class ValidatorTests(unittest.TestCase):
    def setUp(self):
        self.validator = VehicleValidator()

    def parse(self, value):
        return self.validator.parse(value, int(value["host_monotonic_ns"]))

    def test_first_boundary_then_valid_frame_is_immutable_and_keeps_sign(self):
        self.assertEqual(self.parse(row()).reason, "first_sample")
        frame = self.parse(row(2, linear_acc_y_mps2=-2.5))
        self.assertTrue(frame.valid)
        self.assertEqual((frame.ay_mps2, frame.vehicle_id, frame.received_monotonic_ns), (-2.5, "24193", 1_040_000_000))
        with self.assertRaises(FrozenInstanceError):
            frame.ay_mps2 = 0.

    def test_synthetic_needs_explicit_opt_in(self):
        self.assertEqual(self.parse(row(source_kind="synthetic_test")).reason, "unsupported_source")
        validator = VehicleValidator(allow_synthetic=True)
        for seq in (1, 2):
            value = row(seq, source_kind="synthetic_test")
            frame = validator.parse(value, value["host_monotonic_ns"])
        self.assertTrue(frame.valid)
        with self.assertRaises(ValueError):
            VehicleValidator(allow_synthetic=1)

    def test_age_uses_original_receive_clock_and_future_never_advances_state(self):
        self.parse(row())
        stale = row(2)
        self.assertEqual(self.validator.parse(stale, stale["host_monotonic_ns"]+250_000_001).reason, "stale")
        future = row(999)
        self.assertEqual(self.validator.parse(future, 1_100_000_000).reason, "future_timestamp")
        self.assertTrue(self.parse(row(2)).valid)

    def test_host_gap_invalidates_even_when_sim_dt_is_small_then_recovers(self):
        self.parse(row())
        gap = row(2, time_ns=1_400_000_000, sim=.04)
        self.assertEqual(self.parse(gap).reason, "host_gap")
        self.assertTrue(self.parse(row(3, time_ns=1_420_000_000, sim=.06)).valid)

    def test_stream_and_session_boundaries_recover_and_old_stream_is_rejected(self):
        self.parse(row())
        self.assertTrue(self.parse(row(2)).valid)
        self.assertEqual(self.parse(row(0, stream="B", time_ns=1_060_000_000, sim=.01)).reason, "stream_changed")
        self.assertTrue(self.parse(row(1, stream="B", time_ns=1_080_000_000, sim=.03)).valid)
        self.assertEqual(self.parse(row(3, time_ns=1_100_000_000)).reason, "retired_stream")
        self.assertEqual(self.parse(row(2, stream="B", time_ns=1_100_000_000, sim=.05,
                                       _bridge_session_id="new-session")).reason, "stream_changed")

    def test_sequence_reorder_and_gap_do_not_refresh_old_samples(self):
        self.parse(row())
        self.assertEqual(self.parse(row()).reason, "duplicate_or_reordered")
        self.assertEqual(self.parse(row(4)).reason, "sequence_gap")
        self.assertTrue(self.parse(row(5)).valid)
        self.assertEqual(self.parse(row(6, missing_packets_before=1)).reason, "source_packet_gap")

    def test_sim_reset_gap_and_bad_sample_interval_are_invalid(self):
        self.parse(row())
        self.assertEqual(self.parse(row(2, sim=.01)).reason, "nonincreasing_sim_time")
        self.assertTrue(self.parse(row(3, sim=.03)).valid)
        self.assertEqual(self.parse(row(4, sim=1.)).reason, "long_sim_gap")
        self.assertEqual(self.parse(row(5, sim=1.02, sample_dt_s=.3)).reason, "invalid_sample_interval")

    def test_nonfinite_missing_boolean_and_stopped_are_never_filled_with_zero(self):
        self.parse(row())
        cases = [({"linear_acc_y_mps2": None}, "missing:ay_mps2"),
            ({"linear_acc_y_mps2": "NaN"}, "invalid_number:linear_acc_y_mps2"),
            ({"linear_acc_x_mps2": float("inf")}, "invalid_number:linear_acc_x_mps2"),
            ({"speed_mps": True}, "invalid_number:speed_mps"),
            ({"speed_mps": .1}, "stopped"), ({"speed_mps": -1.}, "negative_speed")]
        for seq, (changes, reason) in enumerate(cases, 2):
            with self.subTest(reason=reason):
                frame = self.parse(row(seq, **changes))
                self.assertFalse(frame.valid)
                self.assertEqual(frame.reason, reason)
                if reason.startswith(("missing", "invalid_number")):
                    self.assertTrue(frame.ay_mps2 is None or frame.ax_mps2 is None or frame.speed_mps is None)

    def test_quality_flags_and_motion_bounds_override_collector_ok(self):
        self.parse(row())
        cases = [({"config_match": 0}, "config_mismatch"), ({"config_match": True}, "invalid_quality_flags"),
                 ({"derivative_valid": 0}, "invalid_derivative"),
                 ({"derivative_reason": "rotation_jump"}, "invalid_derivative"),
                 ({"linear_acc_y_mps2": 12.001}, "lateral_acceleration_outlier"),
                 ({"linear_acc_x_mps2": -15.001}, "longitudinal_acceleration_outlier")]
        for seq, (changes, reason) in enumerate(cases, 2):
            self.assertEqual(self.parse(row(seq, **changes)).reason, reason)
        self.assertTrue(self.parse(row(8, linear_acc_y_mps2=-12., linear_acc_x_mps2=15.)).valid)

    def test_config_identity_change_and_explicit_reset(self):
        self.parse(row(_bridge_config_sha256="a"*64))
        self.assertEqual(self.parse(row(2, _bridge_config_sha256="b"*64)).reason, "config_changed")
        self.assertTrue(self.parse(row(3, _bridge_config_sha256="b"*64)).valid)
        self.validator.reset()
        self.assertEqual(self.parse(row(4)).reason, "first_sample")

    def test_stationary_samples_cannot_hide_stream_clock_or_config_boundaries(self):
        self.assertEqual(self.parse(row(speed_mps=0.)).reason, "first_sample")
        self.assertEqual(self.parse(row(2, speed_mps=0.)).reason, "stopped")
        self.assertEqual(self.parse(row(0, stream="B", time_ns=1_060_000_000, sim=.01,
                                        speed_mps=0.)).reason, "stream_changed")
        self.assertEqual(self.parse(row(1, stream="B", time_ns=1_400_000_000, sim=.03,
                                        speed_mps=0.)).reason, "host_gap")

    def test_essential_clock_identity_errors_raise_instead_of_inventing_values(self):
        for changes in ({"host_monotonic_ns": 1.2}, {"host_monotonic_ns": True},
                        {"sequence_id": -1}, {"sim_time_s": None}, {"vehicle_id": None}, {"stream_id": ""}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.validator.parse(row(**changes), 2_000_000_000)


class BridgeTests(unittest.TestCase):
    def test_envelope_preserves_receive_time_and_invalid_derivative_missing_acceleration(self):
        value = row()
        payload = bridge.make_packet(value, provenance(), forwarded_ns=2_000_000_000)
        restored = read_bridge_packet(payload)
        self.assertEqual(restored["host_monotonic_ns"], value["host_monotonic_ns"])
        self.assertEqual(restored["_bridge_session_id"], "offline-session-1")
        self.assertEqual(VehicleValidator().parse(restored, 2_000_000_000).reason, "stale")
        del value["linear_acc_y_mps2"]
        del value["linear_acc_x_mps2"]
        value.update(derivative_valid=0, derivative_reason="first_sample", sample_dt_s=None)
        restored = read_bridge_packet(bridge.make_packet(value, provenance(), forwarded_ns=1_030_000_000))
        self.assertEqual(VehicleValidator().parse(restored, 1_030_000_000).reason, "invalid_derivative")

    def test_envelope_rejects_nonfinite_duplicate_keys_wrong_versions_clocks_and_size(self):
        good = json.loads(bridge.make_packet(row(), provenance(), forwarded_ns=1_030_000_000))
        for changes in ({"schema": "old"}, {"collector_version": "0.2"}, {"csv_revision": True},
                        {"clock": "UTC"}, {"forwarded_monotonic_ns": 0}, {"provenance": {}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                read_bridge_packet(json.dumps(dict(good, **changes)).encode())
        for payload in (b'{"schema":"a","schema":"b"}', b'{"a":NaN}', b'{"a":1e999}',
                        b'[]', b'\xff', b"x"*(MAX_DATAGRAM_BYTES+1)):
            with self.assertRaises(ValueError):
                read_bridge_packet(payload)

    def test_receiver_is_exclusive_loopback64257_and_uses_fake_socket_only(self):
        fake = FakeSocket()
        with BridgeReceiver(socket_factory=lambda *a: fake) as receiver:
            self.assertEqual(fake.bound, ("127.0.0.1", 64257))
            self.assertFalse(fake.blocking)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.assertIn((socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1), fake.options)
            self.assertFalse(any(option[1] == socket.SO_REUSEADDR for option in fake.options))
            self.assertIsNone(receiver.receive())
            fake.incoming.append((bridge.make_packet(row(), provenance(), forwarded_ns=1_030_000_000), (BRIDGE_HOST, 12345)))
            self.assertEqual(receiver.receive()["sequence_id"], 1)
            self.assertEqual(fake.last_receive_limit, MAX_DATAGRAM_BYTES+1)
            fake.incoming.append((b"x"*(MAX_DATAGRAM_BYTES+1), (BRIDGE_HOST, 12345)))
            with self.assertRaises(ValueError):
                receiver.receive()
        self.assertTrue(fake.closed)

    def test_forwarder_is_nonblocking_drops_errors_and_never_binds_collector_port(self):
        fake = FakeSocket()
        sender = bridge.UDPForwarder(socket_factory=lambda *a: fake, clock=lambda: 1_030_000_000)
        self.assertTrue(sender.publish(row(), provenance()))
        self.assertEqual(fake.sent[0][1], (BRIDGE_HOST, BRIDGE_PORT))
        self.assertIsNone(fake.bound)
        self.assertFalse(fake.blocking)
        fake.send_error = BlockingIOError("buffer full")
        self.assertFalse(sender.publish(row(), provenance()))
        self.assertEqual((sender.sent, sender.dropped), (1, 1))
        self.assertIn("buffer full", sender.last_error)
        sender.close()
        self.assertTrue(fake.closed)

    def test_session_observer_runs_after_archive_and_never_throttles_or_breaks_return(self):
        trace = []
        class Original:
            path = Path("offline-session")
            c = {"source": "synthetic unit test"}
            def accept(self, payload, mono=None, wall=None):
                trace.append(("archived", payload))
                return payload
        class Sender:
            def publish(self, value, context):
                self.last = context
                trace.append(("forwarded", value))
                if value.get("sequence_id") == 5:
                    raise RuntimeError("observer crashed")
        sender = Sender()
        session = bridge.observed_session_class(Original, sender, provenance())()
        for seq in range(1, 11):
            value = row(seq)
            self.assertIs(session.accept(value, value["host_monotonic_ns"], 123), value)
        self.assertEqual([name for name, value in trace], ["archived", "forwarded"]*10)
        self.assertIsNone(session.accept(None))
        self.assertIn("observer crashed", session._fallback_observer_error)
        self.assertEqual(sender.last["config_sha256"], bridge.configuration_sha256(Original.c))

    def test_launcher_restores_in_memory_patch_bytecode_and_import_path_without_gui_socket(self):
        trace = []
        class Original:
            path = Path("offline-session")
            c = {}
            def accept(self, payload, mono=None, wall=None):
                trace.append("archive")
                return payload
        fake_socket = FakeSocket()
        sender = bridge.UDPForwarder(socket_factory=lambda *a: fake_socket, clock=lambda: 1_030_000_000)
        with tempfile.TemporaryDirectory() as tmp:
            module = SimpleNamespace(__file__=str(Path(tmp)/"desktop_app.py"), Session=Original)
            def fake_main():
                self.assertTrue(sys.dont_write_bytecode)
                self.assertIsNot(module.Session, Original)
                module.Session().accept(row())
            module.main = fake_main
            context = dict(provenance(), collector_root=tmp)
            old_path, old_bytecode = list(sys.path), sys.dont_write_bytecode
            with patch.object(bridge.socket, "socket", side_effect=AssertionError("Real socket forbidden")):
                report = bridge.run_collector(tmp, publisher_factory=lambda: sender,
                    importer=lambda name: module, verifier=lambda root: context)
            self.assertEqual(report["sent"], 1)
            self.assertIs(module.Session, Original)
            self.assertEqual(sys.path, old_path)
            self.assertEqual(sys.dont_write_bytecode, old_bytecode)
            self.assertTrue(fake_socket.closed)
            self.assertEqual(list(Path(tmp).iterdir()), [])

    def test_observer_socket_start_failure_still_runs_archive(self):
        with tempfile.TemporaryDirectory() as tmp:
            class Original:
                path, c = Path(tmp)/"session", {}
                def accept(self, payload, mono=None, wall=None):
                    return payload
            module = SimpleNamespace(__file__=str(Path(tmp)/"desktop_app.py"), Session=Original)
            module.main = lambda: module.Session().accept(row())
            def fail():
                raise OSError("observer socket unavailable")
            with contextlib.redirect_stderr(io.StringIO()):
                report = bridge.run_collector(tmp, publisher_factory=fail, importer=lambda n: module,
                    verifier=lambda p: dict(provenance(), collector_root=tmp))
            self.assertEqual((report["sent"], report["dropped"]), (0, 1))
            self.assertIs(module.Session, Original)

    def test_csv_keeps_exact_integer_timestamp_and_blank_fields(self):
        value = row(time_ns=9_007_199_254_740_993, linear_acc_y_mps2=None)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"vehicle.csv"
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(value))
                writer.writeheader()
                writer.writerow(value)
            before = path.read_bytes()
            restored = list(read_vehicle_csv(path))[0]
            self.assertEqual(restored["host_monotonic_ns"], "9007199254740993")
            self.assertEqual(restored["linear_acc_y_mps2"], "")
            self.assertEqual(path.read_bytes(), before)


class LocalEvidenceTests(unittest.TestCase):
    @unittest.skipUnless((LOCAL_COLLECTOR/"collect.py").is_file(), "Local collector source is external, not copied into Git")
    def test_actual_v03_contract_is_read_only_and_check_only_opens_nothing(self):
        before = {name: hashlib.sha256((LOCAL_COLLECTOR/name).read_bytes()).hexdigest() for name in bridge.COLLECTOR_FILES}
        with patch.object(bridge, "run_collector", side_effect=AssertionError("GUI forbidden")), \
             patch.object(bridge.socket, "socket", side_effect=AssertionError("Socket forbidden")), \
             contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(bridge.main(["--collector-root", str(LOCAL_COLLECTOR), "--check-only"]), 0)
        report = json.loads(output.getvalue())
        self.assertEqual((report["collector_version"], report["csv_revision"]), ("0.3", 2))
        self.assertEqual(report["source_sha256"], before)
        self.assertEqual(before, {name: hashlib.sha256((LOCAL_COLLECTOR/name).read_bytes()).hexdigest() for name in bridge.COLLECTOR_FILES})

    @unittest.skipUnless(LOCAL_DRIVE.is_file(), "External recorded telemetry is not copied into Git")
    def test_actual_drive_replay_catches_high_acceleration_and_host_gaps_preserving_input(self):
        digest = hashlib.sha256(LOCAL_DRIVE.read_bytes()).hexdigest()
        validator, counts, rows = VehicleValidator(), {}, 0
        for value in read_vehicle_csv(LOCAL_DRIVE):
            frame = validator.parse(value, int(value["host_monotonic_ns"]))
            counts[frame.reason] = counts.get(frame.reason, 0)+1
            rows += 1
        self.assertEqual(rows, 13223)
        self.assertGreater(counts.get("lateral_acceleration_outlier", 0), 0)
        self.assertGreater(counts.get("longitudinal_acceleration_outlier", 0), 0)
        self.assertGreater(counts.get("host_gap", 0), 0)
        self.assertGreater(counts.get("ok", 0), 1000)
        self.assertEqual(hashlib.sha256(LOCAL_DRIVE.read_bytes()).hexdigest(), digest)

    @unittest.skipUnless(LOCAL_STOP.is_file(), "External stationary telemetry is not copied into Git")
    def test_actual_stationary_recording_never_becomes_support_input(self):
        validator, count = VehicleValidator(), 0
        for value in read_vehicle_csv(LOCAL_STOP):
            frame = validator.parse(value, int(value["host_monotonic_ns"]))
            self.assertFalse(frame.valid)
            self.assertIn(frame.reason, ("invalid_derivative", "stopped", "host_gap"))
            count += 1
        self.assertEqual(count, 5219)


if __name__ == "__main__":
    unittest.main(verbosity=2)
