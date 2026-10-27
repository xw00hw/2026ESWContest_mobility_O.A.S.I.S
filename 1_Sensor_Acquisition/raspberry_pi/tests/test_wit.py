"""WIT standard protocol: framing, checksums, scaling, resynchronisation."""

import math

import pytest

from oasis_daq.imu import wit

# Captured verbatim from the attached unit at 9600 baud (od -t x1 /dev/ttyUSB0).
REAL_ACC = bytes.fromhex("555159038efb49fa230afb")
REAL_GYRO = bytes.fromhex("5552f8fff8fffcff230abd")
REAL_ANGLE = bytes.fromhex("5553e69a60ee04b10b477d")
REAL_MAG = bytes.fromhex("5554efd47afa041d000001")
REAL_ROUND = REAL_ACC + REAL_GYRO + REAL_ANGLE + REAL_MAG


def _frames(data):
    return list(wit.FrameParser().feed(data))


def test_real_capture_parses_with_valid_checksums():
    frames = _frames(REAL_ROUND)
    assert len(frames) == 4
    assert all(f.checksum_valid for f in frames)
    assert [f.name for f in frames] == ["acc", "gyro", "angle", "mag"]


def test_checksum_is_low_byte_of_the_sum_of_the_first_ten_bytes():
    assert wit.checksum(REAL_ACC) == REAL_ACC[10]


def test_corrupted_checksum_is_flagged_not_dropped():
    bad = bytearray(REAL_ACC)
    bad[10] ^= 0xFF
    frames = _frames(bytes(bad))
    assert len(frames) == 1 and frames[0].checksum_valid is False


def test_corrupted_payload_is_detected():
    bad = bytearray(REAL_ACC)
    bad[3] ^= 0x01
    assert _frames(bytes(bad))[0].checksum_valid is False


def test_parser_resynchronises_after_garbage():
    parser = wit.FrameParser()
    frames = list(parser.feed(b"\x00\xde\xad\xbe\xef" + REAL_ROUND))
    assert len(frames) == 4
    assert parser.bytes_discarded == 5
    assert parser.frames_bad_checksum == 0


def test_parser_handles_byte_by_byte_delivery():
    parser = wit.FrameParser()
    frames = []
    for i in range(len(REAL_ROUND)):
        frames.extend(parser.feed(REAL_ROUND[i:i + 1]))
    assert len(frames) == 4 and all(f.checksum_valid for f in frames)


def test_a_0x55_inside_a_payload_does_not_desynchronise():
    parser = wit.FrameParser()
    # 0x55 followed by a byte that is not a known packet type is payload noise.
    frames = list(parser.feed(b"\x55\x99" + REAL_ROUND))
    assert len(frames) == 4


def _assemble(data):
    asm = wit.SampleAssembler()
    out = [s for f in wit.FrameParser().feed(data)
           if (s := asm.feed(f, 111, 222)) is not None]
    return out[0] if out else asm.flush(111, 222)


def test_acceleration_magnitude_is_one_g_at_rest():
    s = _assemble(REAL_ROUND)
    magnitude = math.sqrt(s.acc_x_mps2 ** 2 + s.acc_y_mps2 ** 2 + s.acc_z_mps2 ** 2)
    assert magnitude == pytest.approx(wit.G_TO_MPS2, rel=0.01)


def test_scaling_constants_match_the_wit_standard():
    s = _assemble(REAL_ROUND)
    # raw / 32768 * range, then g -> m/s^2
    assert s.acc_x_mps2 == pytest.approx(857 / 32768 * 16 * 9.80665, rel=1e-9)
    assert s.gyro_x_radps == pytest.approx(-8 / 32768 * 2000 * math.pi / 180, rel=1e-9)
    assert s.roll_rad == pytest.approx(-25882 / 32768 * math.pi, rel=1e-9)


def test_temperature_is_plausible_room_temperature():
    assert 15.0 < _assemble(REAL_ROUND).temperature_c < 40.0


def test_raw_counts_are_preserved_alongside_engineering_units():
    s = _assemble(REAL_ROUND)
    assert (s.raw_acc_x, s.raw_acc_y, s.raw_acc_z) == (857, -1138, -1463)
    assert (s.raw_mag_x, s.raw_mag_y, s.raw_mag_z) == (-11025, -1414, 7428)


def test_host_timestamps_are_recorded_on_the_sample():
    s = _assemble(REAL_ROUND)
    assert (s.host_monotonic_ns, s.host_wall_time_ns) == (111, 222)
    assert s.sample_valid is True and s.packets_in_sample == 4


def test_identity_mounting_leaves_vehicle_frame_equal_to_sensor_frame():
    s = _assemble(REAL_ROUND)
    assert s.veh_acc_x_mps2 == pytest.approx(s.acc_x_mps2)
    assert s.veh_acc_z_mps2 == pytest.approx(s.acc_z_mps2)


def test_mounting_rotation_is_applied_to_the_vehicle_frame_only():
    """A 90 deg yaw maps sensor +x onto vehicle +y."""
    rotation = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    asm = wit.SampleAssembler(rotation_matrix=rotation)
    sample = None
    for f in wit.FrameParser().feed(REAL_ROUND):
        sample = asm.feed(f, 0, 0) or sample
    sample = sample or asm.flush(0, 0)
    assert sample.veh_acc_x_mps2 == pytest.approx(-sample.acc_y_mps2)
    assert sample.veh_acc_y_mps2 == pytest.approx(sample.acc_x_mps2)
    # The sensor-frame values must be untouched by the transform.
    assert sample.acc_x_mps2 == pytest.approx(857 / 32768 * 16 * 9.80665)


def test_gyro_bias_calibration_applies_to_vehicle_frame_only():
    bias = [0.5, 0.0, 0.0]
    asm = wit.SampleAssembler(calibration={"gyro_bias_radps": bias})
    sample = None
    for f in wit.FrameParser().feed(REAL_ROUND):
        sample = asm.feed(f, 0, 0) or sample
    sample = sample or asm.flush(0, 0)
    assert sample.veh_gyro_x_radps == pytest.approx(sample.gyro_x_radps - 0.5)


def test_bad_checksum_marks_the_sample_invalid():
    bad = bytearray(REAL_ACC)
    bad[10] ^= 0xFF
    asm = wit.SampleAssembler()
    sample = None
    for f in wit.FrameParser().feed(bytes(bad) + REAL_GYRO + REAL_ANGLE + REAL_MAG):
        sample = asm.feed(f, 0, 0) or sample
    sample = sample or asm.flush(0, 0)
    assert sample.checksum_valid is False and sample.sample_valid is False


def test_repeated_packet_type_closes_the_previous_sample():
    asm = wit.SampleAssembler()
    emitted = [s for f in wit.FrameParser().feed(REAL_ACC + REAL_GYRO + REAL_ACC)
               if (s := asm.feed(f, 0, 0)) is not None]
    assert len(emitted) == 1 and emitted[0].packets_in_sample == 2


def test_csv_fields_cover_the_documented_schema():
    required = {
        "host_monotonic_ns", "host_wall_time_ns", "sensor_sequence",
        "raw_acc_x", "raw_gyro_x", "raw_mag_x", "raw_roll", "temperature_c",
        "acc_x_mps2", "gyro_x_radps", "mag_x", "roll_rad", "quaternion_w",
        "packet_type", "checksum_valid", "sample_valid",
    }
    assert required <= set(wit.CSV_FIELDS)


# --- mounting rotation validation ---------------------------------------
def test_identity_rotation_is_accepted_and_is_the_default():
    assert wit.validate_rotation(None) == wit.IDENTITY
    assert wit.validate_rotation(wit.IDENTITY) == wit.IDENTITY


def test_a_proper_rotation_is_accepted():
    yaw90 = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    assert wit.validate_rotation(yaw90) == yaw90


def test_a_mirrored_matrix_is_rejected():
    """det = -1 mirrors the vehicle frame and is invisible in the data."""
    mirrored = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]
    with pytest.raises(wit.InvalidRotation, match="determinant"):
        wit.validate_rotation(mirrored)


def test_a_non_orthonormal_matrix_is_rejected():
    with pytest.raises(wit.InvalidRotation, match="orthonormal"):
        wit.validate_rotation([[2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


@pytest.mark.parametrize("bad", [
    [[1.0, 0.0], [0.0, 1.0]],                       # 2x2
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],             # only two rows
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0]], # ragged
    "not a matrix",
    [["a", "b", "c"], [0, 1, 0], [0, 0, 1]],
])
def test_malformed_matrices_are_rejected_at_construction(bad):
    """These used to raise IndexError/TypeError deep inside the acquisition
    loop instead of at startup where the operator can act."""
    with pytest.raises(wit.InvalidRotation):
        wit.validate_rotation(bad)


def test_assembler_rejects_a_bad_rotation_up_front():
    with pytest.raises(wit.InvalidRotation):
        wit.SampleAssembler(rotation_matrix=[[1.0, 0.0], [0.0, 1.0]])
