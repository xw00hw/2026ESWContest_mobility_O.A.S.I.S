"""NMEA parsing: checksums, coordinate conversion, unit conversion, assembly."""

import math

import pytest

from oasis_daq.gps import nmea

GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
VTG = "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48"
GSA = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39"
# Captured verbatim from the attached u-blox 7 with no fix (indoors).
NOFIX_RMC = "$GPRMC,,V,,,,,,,,,,N*53"
NOFIX_GGA = "$GPGGA,,,,,,0,00,99.99,,,,,,*48"


@pytest.mark.parametrize("sentence", [GGA, RMC, VTG, GSA, NOFIX_RMC, NOFIX_GGA])
def test_checksum_accepts_valid(sentence):
    ok, computed, declared = nmea.verify_checksum(sentence)
    assert ok and computed == declared


def test_checksum_rejects_corrupted_payload():
    corrupted = GGA.replace("545.4", "545.5")
    assert nmea.verify_checksum(corrupted)[0] is False


def test_checksum_rejects_wrong_declared_value():
    assert nmea.verify_checksum(GGA[:-2] + "48")[0] is False


def test_checksum_rejects_missing_suffix():
    ok, _, declared = nmea.verify_checksum("$GPGGA,123519")
    assert ok is False and declared is None


def test_latitude_longitude_conversion():
    fix = nmea.GpsAssembler().feed(GGA, 0, 0)
    # 4807.038 N -> 48 deg + 7.038/60 min
    assert fix.latitude_deg == pytest.approx(48 + 7.038 / 60, abs=1e-9)
    assert fix.longitude_deg == pytest.approx(11 + 31.0 / 60, abs=1e-9)


def test_southern_and_western_hemispheres_are_negative():
    south = "$GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,"
    south += f"*{nmea.verify_checksum(south + '*00')[1]}"
    fix = nmea.GpsAssembler().feed(south, 0, 0)
    assert fix.latitude_deg < 0 and fix.longitude_deg < 0


def test_speed_converted_from_knots_to_mps():
    fix = nmea.GpsAssembler().feed(RMC, 0, 0)
    assert fix.ground_speed_mps == pytest.approx(22.4 * 0.514444, rel=1e-9)


def test_rmc_course_comes_from_field_seven_not_the_date():
    """Regression: course and date are adjacent fields and easy to swap."""
    fix = nmea.GpsAssembler().feed(RMC, 0, 0)
    assert fix.course_deg == pytest.approx(84.4)
    assert fix.gps_utc == "1994-03-23T12:35:19.000Z"


def test_gga_hdop_propagates_to_the_following_rmc_row():
    asm = nmea.GpsAssembler()
    asm.feed(GGA, 0, 0)
    fix = asm.feed(RMC, 1, 1)
    assert fix.hdop == pytest.approx(0.9)


def test_gsa_supplies_the_dop_triple_to_later_rows():
    asm = nmea.GpsAssembler()
    asm.feed(GSA, 0, 0)
    fix = asm.feed(GGA, 1, 1)
    assert (fix.pdop, fix.vdop) == (pytest.approx(2.5), pytest.approx(2.1))


def test_vtg_updates_speed_of_the_current_row():
    asm = nmea.GpsAssembler()
    asm.feed(GGA, 0, 0)
    asm.feed(VTG, 1, 1)
    assert asm.last_fix.ground_speed_mps == pytest.approx(5.5 * 0.514444, rel=1e-9)


def test_no_fix_sentences_do_not_fabricate_a_position():
    asm = nmea.GpsAssembler()
    for sentence in (NOFIX_RMC, NOFIX_GGA):
        fix = asm.feed(sentence, 0, 0)
        assert fix is not None
        assert fix.fix_valid is False
        assert fix.latitude_deg is None and fix.longitude_deg is None
    assert asm.checksum_failures == 0


def test_corrupted_sentence_is_counted_and_produces_no_row():
    asm = nmea.GpsAssembler()
    assert asm.feed(GGA.replace("545.4", "545.5"), 0, 0) is None
    assert asm.checksum_failures == 1


def test_garbage_never_raises():
    asm = nmea.GpsAssembler()
    for junk in ("", "   ", "not nmea", "$", "$GP*", "$GPGGA," * 40, "\x00\xff"):
        asm.feed(junk, 0, 0)          # must not raise


def test_out_of_range_minutes_are_rejected():
    bad = "$GPGGA,123519,4867.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
    bad += f"*{nmea.verify_checksum(bad + '*00')[1]}"
    fix = nmea.GpsAssembler().feed(bad, 0, 0)
    assert fix.latitude_deg is None


def test_host_timestamps_and_sequence_are_recorded():
    asm = nmea.GpsAssembler()
    first = asm.feed(GGA, 111, 222)
    second = asm.feed(RMC, 333, 444)
    assert (first.host_monotonic_ns, first.host_wall_time_ns) == (111, 222)
    assert (second.sequence_id - first.sequence_id) == 1


def test_csv_fields_cover_the_documented_schema():
    required = {
        "host_monotonic_ns", "host_wall_time_ns", "gps_utc", "latitude_deg",
        "longitude_deg", "altitude_m", "ground_speed_mps", "course_deg",
        "fix_quality", "fix_valid", "satellite_count", "hdop", "vdop", "pdop",
        "source_sentence", "sequence_id",
    }
    assert required <= set(nmea.CSV_FIELDS)


# --- carried-forward fields ---------------------------------------------
def test_rmc_row_declares_the_fields_it_inherited_from_an_earlier_gga():
    """This receiver emits RMC first in each epoch, so an RMC row's altitude
    and fix_quality come from the PREVIOUS second's GGA. The row must say so."""
    asm = nmea.GpsAssembler()
    asm.feed(GGA, 0, 0)                      # epoch 1 GGA: altitude, quality
    row = asm.feed(RMC, 1, 1)                # epoch 2 RMC (in real order)
    assert row.altitude_m == pytest.approx(545.4)
    carried = row.carried_fields.split("|")
    assert "altitude_m" in carried
    assert "fix_quality" in carried
    assert "satellite_count" in carried


def test_gga_row_declares_speed_and_course_as_carried():
    asm = nmea.GpsAssembler()
    asm.feed(RMC, 0, 0)
    row = asm.feed(GGA, 1, 1)
    carried = row.carried_fields.split("|")
    assert "ground_speed_mps" in carried and "course_deg" in carried


def test_the_first_row_of_a_session_carries_nothing():
    assert nmea.GpsAssembler().feed(GGA, 0, 0).carried_fields == ""


def test_carried_fields_is_part_of_the_csv_schema():
    assert "carried_fields" in nmea.CSV_FIELDS
