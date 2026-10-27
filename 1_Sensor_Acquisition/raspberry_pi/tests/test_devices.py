"""Device identity resolution: never guess, never accept an ambiguous match."""

import pytest

from oasis_daq import devices as dev


def serial(device, by_id=None, vid=None, pid=None, serial_no=None):
    return dev.SerialDeviceInfo(device=device, by_id=by_id, vendor_id=vid,
                                product_id=pid, serial=serial_no)


def camera(device, card=None, vid=None, pid=None):
    return dev.CameraDeviceInfo(device=device, card=card, vendor_id=vid,
                                product_id=pid)


# Modelled on what is actually attached to the bench Pi.
GPS = serial("/dev/ttyACM0",
             by_id="/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_"
                   "u-blox_7_-_GPS_GNSS_Receiver-if00",
             vid="1546", pid="01a7")
IMU = serial("/dev/ttyUSB0",
             by_id="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
             vid="1a86", pid="7523")
CAM = camera("/dev/video0", card="Arducam OV9281 USB Camera: Ardu",
             vid="0c45", pid="6366")


def test_matcher_selects_the_right_serial_device():
    m = dev.DeviceMatcher(vid_pid="1546:01a7", by_id_contains="u-blox")
    path, info = dev.resolve_serial(m, "/nonexistent", candidates=[GPS, IMU])
    assert info is GPS
    # by-id is preferred over the kernel name, because ttyACM0 is not an identity
    assert path == GPS.by_id


def test_resolution_prefers_by_id_over_the_kernel_name():
    assert GPS.stable_path.startswith("/dev/serial/by-id/")


def test_by_path_is_used_when_there_is_no_by_id():
    d = dev.SerialDeviceInfo(device="/dev/ttyUSB9",
                             by_path="/dev/serial/by-path/platform-x-usb-0:1:1.0")
    assert d.stable_path == d.by_path


def test_wrong_vid_pid_never_matches():
    m = dev.DeviceMatcher(vid_pid="1546:01a7")
    with pytest.raises(dev.DeviceNotFound):
        dev.resolve_serial(m, "/nonexistent", candidates=[IMU])


def test_an_empty_matcher_refuses_to_guess():
    """With no udev link and no criteria, picking 'the only device' is wrong."""
    with pytest.raises(dev.DeviceNotFound):
        dev.resolve_serial(dev.DeviceMatcher(), "/nonexistent", candidates=[GPS])


def test_an_ambiguous_matcher_is_an_error_not_a_coin_flip():
    """Two CH340 adapters is a real scenario -- the generic id must not silently
    pick one."""
    second_ch340 = serial("/dev/ttyUSB1",
                          by_id="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port1",
                          vid="1a86", pid="7523")
    m = dev.DeviceMatcher(vid_pid="1a86:7523")
    with pytest.raises(dev.DeviceNotFound, match="ambiguous"):
        dev.resolve_serial(m, "/nonexistent", candidates=[IMU, second_ch340])


def test_serial_number_disambiguates_identical_adapters():
    a = serial("/dev/ttyUSB0", vid="1a86", pid="7523", serial_no="AAA")
    b = serial("/dev/ttyUSB1", vid="1a86", pid="7523", serial_no="BBB")
    m = dev.DeviceMatcher(vid_pid="1a86:7523", serial="BBB")
    _, info = dev.resolve_serial(m, "/nonexistent", candidates=[a, b])
    assert info is b


def test_error_message_lists_what_was_actually_present():
    m = dev.DeviceMatcher(vid_pid="dead:beef")
    with pytest.raises(dev.DeviceNotFound) as exc:
        dev.resolve_serial(m, "/nonexistent", candidates=[GPS, IMU])
    assert "1546:01a7" in str(exc.value) and "1a86:7523" in str(exc.value)


def test_camera_matches_on_the_v4l2_card_name():
    m = dev.DeviceMatcher(vid_pid="0c45:6366", card_contains="Arducam OV9281")
    path, info = dev.resolve_camera(m, "/nonexistent", candidates=[CAM])
    assert path == "/dev/video0" and info is CAM


def test_camera_matcher_is_case_insensitive():
    m = dev.DeviceMatcher(card_contains="arducam ov9281")
    _, info = dev.resolve_camera(m, "/nonexistent", candidates=[CAM])
    assert info is CAM


def test_camera_metadata_node_is_not_selected():
    """A UVC camera exposes several video nodes; only the capture node has a
    card name we match, and enumeration drops nodes with no formats."""
    meta = camera("/dev/video1", card="Arducam OV9281 USB Camera: Ardu",
                  vid="0c45", pid="6366")
    m = dev.DeviceMatcher(vid_pid="0c45:6366", card_contains="Arducam")
    with pytest.raises(dev.DeviceNotFound, match="ambiguous"):
        dev.resolve_camera(m, "/nonexistent", candidates=[CAM, meta])


def test_matcher_from_config_ignores_blank_fields():
    m = dev.DeviceMatcher.from_config({"vid_pid": "1546:01a7", "device": "",
                                       "by_id_contains": ""})
    assert m.vid_pid == "1546:01a7"
    assert m.by_id_contains is None and m.device is None
    assert not m.is_empty()


def test_from_config_on_an_empty_mapping_is_empty():
    assert dev.DeviceMatcher.from_config(None).is_empty()
    assert dev.DeviceMatcher.from_config({}).is_empty()


def test_vid_pid_is_formatted_as_vendor_colon_product():
    assert GPS.vid_pid == "1546:01a7"
    assert dev.SerialDeviceInfo(device="/dev/x").vid_pid is None


def test_to_dict_exposes_the_derived_fields_for_metadata():
    d = GPS.to_dict()
    assert d["vid_pid"] == "1546:01a7" and d["stable_path"] == GPS.by_id


# --- the udev symlink must not be trusted blindly -----------------------
def test_udev_symlink_is_used_when_it_satisfies_the_matcher(tmp_path, monkeypatch):
    link = tmp_path / "oasis_gps"
    link.symlink_to("/dev/ttyACM0")
    path, info = dev.resolve_serial(dev.DeviceMatcher(vid_pid="1546:01a7"),
                                    str(link), candidates=[GPS, IMU])
    assert path == str(link) and info is GPS


def test_udev_symlink_is_rejected_when_it_contradicts_the_matcher(tmp_path):
    """Regression: the symlink was taken on faith. If the udev rules and the
    config ever disagree, following the link silently opens the wrong device."""
    link = tmp_path / "oasis_gps"
    link.symlink_to("/dev/ttyUSB0")          # points at the IMU, not the GPS
    with pytest.raises(dev.DeviceNotFound):
        dev.resolve_serial(dev.DeviceMatcher(vid_pid="1546:01a7"),
                           str(link), candidates=[IMU])


def test_a_dangling_udev_symlink_falls_back_to_identity_matching(tmp_path):
    """A stale symlink left behind by an unplugged device must not break
    resolution: os.path.exists() follows the link, so it is skipped and the
    VID/PID match still finds the right device."""
    link = tmp_path / "oasis_gps"
    link.symlink_to("/dev/ttyNOTINVENTORY")
    path, info = dev.resolve_serial(dev.DeviceMatcher(vid_pid="1546:01a7"),
                                    str(link), candidates=[GPS, IMU])
    assert info is GPS and path == GPS.by_id


def test_udev_symlink_to_a_device_outside_the_inventory_is_not_trusted(tmp_path):
    """If the link resolves but we cannot identify what it points at, following
    it would be exactly the blind trust this check exists to prevent."""
    target = tmp_path / "ttyMYSTERY"
    target.write_text("")
    link = tmp_path / "oasis_gps"
    link.symlink_to(target)
    with pytest.raises(dev.DeviceNotFound):
        # GPS/IMU are the known inventory; the link points outside it.
        dev.resolve_serial(dev.DeviceMatcher(vid_pid="1546:01a7"),
                           str(link), candidates=[IMU])


def test_an_explicit_device_override_always_wins(tmp_path):
    """An operator writing `device:` in the config is being deliberate."""
    node = tmp_path / "ttyFAKE"
    node.write_text("")
    m = dev.DeviceMatcher(vid_pid="1546:01a7", device=str(node))
    path, _ = dev.resolve_serial(m, "/nonexistent", candidates=[GPS, IMU])
    assert path == str(node)


def test_camera_udev_symlink_is_verified_too(tmp_path):
    link = tmp_path / "oasis_camera"
    link.symlink_to("/dev/video0")
    path, info = dev.resolve_camera(dev.DeviceMatcher(card_contains="Arducam"),
                                    str(link), candidates=[CAM])
    assert path == str(link) and info is CAM
    with pytest.raises(dev.DeviceNotFound):
        dev.resolve_camera(dev.DeviceMatcher(card_contains="Logitech"),
                           str(link), candidates=[CAM])
