"""Stable hardware discovery for OASIS DAQ.

Enumeration never trusts ``/dev/ttyUSB0`` / ``/dev/video0`` as an identity:
those names depend on probe order.  Devices are resolved by USB VID/PID plus
``/dev/serial/by-id`` (serial links) or the V4L2 card name (camera), and the
resolved identity is written into ``metadata.json`` for every session.
"""

from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
from dataclasses import dataclass, field, asdict
from typing import Iterable

__all__ = [
    "SerialDeviceInfo",
    "CameraDeviceInfo",
    "DeviceMatcher",
    "list_serial_devices",
    "list_video_capture_devices",
    "resolve_serial",
    "resolve_camera",
    "snapshot",
]

# Stable symlinks installed by udev/99-oasis-daq.rules.  Preferred when present.
UDEV_GPS = "/dev/oasis_gps"
UDEV_IMU = "/dev/oasis_imu"
UDEV_CAMERA = "/dev/oasis_camera"

_UDEVADM = shutil.which("udevadm")
_V4L2_CTL = shutil.which("v4l2-ctl")


@dataclass
class SerialDeviceInfo:
    """A USB serial port together with everything that identifies it."""

    device: str                      # resolved real path, e.g. /dev/ttyACM0
    by_id: str | None = None         # /dev/serial/by-id/... (stable across reboots)
    by_path: str | None = None       # /dev/serial/by-path/... (stable per USB port)
    vendor_id: str | None = None
    product_id: str | None = None
    vendor: str | None = None
    product: str | None = None
    serial: str | None = None
    driver: str | None = None
    usb_path: str | None = None

    @property
    def vid_pid(self) -> str | None:
        if self.vendor_id and self.product_id:
            return f"{self.vendor_id}:{self.product_id}"
        return None

    @property
    def stable_path(self) -> str:
        """Path a worker should open: prefer by-id, fall back to by-path."""
        return self.by_id or self.by_path or self.device

    def to_dict(self) -> dict:
        d = asdict(self)
        d["vid_pid"] = self.vid_pid
        d["stable_path"] = self.stable_path
        return d


@dataclass
class CameraDeviceInfo:
    """A V4L2 video-capture node plus its USB identity and formats."""

    device: str
    card: str | None = None
    driver: str | None = None
    bus_info: str | None = None
    vendor_id: str | None = None
    product_id: str | None = None
    serial: str | None = None
    by_id: str | None = None
    formats: list[dict] = field(default_factory=list)

    @property
    def vid_pid(self) -> str | None:
        if self.vendor_id and self.product_id:
            return f"{self.vendor_id}:{self.product_id}"
        return None

    @property
    def stable_path(self) -> str:
        return self.by_id or self.device

    def to_dict(self) -> dict:
        d = asdict(self)
        d["vid_pid"] = self.vid_pid
        d["stable_path"] = self.stable_path
        return d


@dataclass
class DeviceMatcher:
    """Config-driven predicate used to pick one device out of many.

    Every populated field must match.  ``by_id_contains`` and ``card_contains``
    are case-insensitive substring tests; ``vid_pid`` is an exact match.
    """

    vid_pid: str | None = None
    by_id_contains: str | None = None
    card_contains: str | None = None
    serial: str | None = None
    device: str | None = None          # explicit override, used verbatim

    @classmethod
    def from_config(cls, cfg: dict | None) -> "DeviceMatcher":
        cfg = cfg or {}
        return cls(
            vid_pid=(cfg.get("vid_pid") or None),
            by_id_contains=(cfg.get("by_id_contains") or None),
            card_contains=(cfg.get("card_contains") or None),
            serial=(cfg.get("serial") or None),
            device=(cfg.get("device") or None),
        )

    def _device_matches(self, device: str) -> bool:
        """`device:` is an identity too -- ignoring it here made a matcher that
        carried only `device:` non-empty yet true for every port."""
        if not self.device:
            return True
        return os.path.realpath(self.device) == os.path.realpath(device)

    def matches_serial(self, info: SerialDeviceInfo) -> bool:
        if not self._device_matches(info.device):
            return False
        if self.card_contains:
            return False        # a card name cannot match a serial port
        if self.vid_pid and (info.vid_pid or "").lower() != self.vid_pid.lower():
            return False
        if self.by_id_contains:
            hay = (info.by_id or "").lower()
            if self.by_id_contains.lower() not in hay:
                return False
        if self.serial and (info.serial or "") != self.serial:
            return False
        return True

    def matches_camera(self, info: CameraDeviceInfo) -> bool:
        if not self._device_matches(info.device):
            return False
        if self.by_id_contains:
            return False        # a serial by-id path cannot match a camera
        if self.vid_pid and (info.vid_pid or "").lower() != self.vid_pid.lower():
            return False
        if self.card_contains:
            hay = (info.card or "").lower()
            if self.card_contains.lower() not in hay:
                return False
        if self.serial and (info.serial or "") != self.serial:
            return False
        return True

    def is_empty(self) -> bool:
        return not any((self.vid_pid, self.by_id_contains, self.card_contains,
                        self.serial, self.device))


def _udev_properties(device: str) -> dict[str, str]:
    """Read udev properties for a device node; empty dict if udevadm is absent."""
    if not _UDEVADM or not os.path.exists(device):
        return {}
    try:
        out = subprocess.run(
            [_UDEVADM, "info", "--query=property", f"--name={device}"],
            capture_output=True, text=True, timeout=5, check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return {}
    props: dict[str, str] = {}
    for line in out.splitlines():
        if "=" in line:
            key, _, value = line.partition("=")
            props[key.strip()] = value.strip()
    return props


def _link_map(pattern: str) -> dict[str, str]:
    """Map realpath -> symlink for every link matching *pattern*."""
    mapping: dict[str, str] = {}
    for link in sorted(glob.glob(pattern)):
        try:
            mapping.setdefault(os.path.realpath(link), link)
        except OSError:
            continue
    return mapping


def list_serial_devices() -> list[SerialDeviceInfo]:
    """Every USB/UART serial port currently present, with stable identities."""
    by_id = _link_map("/dev/serial/by-id/*")
    by_path = _link_map("/dev/serial/by-path/*")

    nodes: set[str] = set()
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        nodes.update(glob.glob(pattern))
    nodes.update(by_id.keys())

    devices: list[SerialDeviceInfo] = []
    for node in sorted(nodes):
        props = _udev_properties(node)
        devices.append(
            SerialDeviceInfo(
                device=node,
                by_id=by_id.get(node),
                by_path=by_path.get(node),
                vendor_id=props.get("ID_VENDOR_ID"),
                product_id=props.get("ID_MODEL_ID"),
                vendor=props.get("ID_VENDOR_FROM_DATABASE") or props.get("ID_VENDOR"),
                product=props.get("ID_MODEL_FROM_DATABASE") or props.get("ID_MODEL"),
                serial=props.get("ID_SERIAL_SHORT"),
                driver=props.get("ID_USB_DRIVER"),
                usb_path=props.get("ID_PATH"),
            )
        )
    return devices


def _v4l2_capabilities(device: str) -> tuple[str | None, str | None, str | None]:
    """Return ``(card, driver, bus_info)`` from ``v4l2-ctl --all``."""
    if not _V4L2_CTL:
        return None, None, None
    try:
        out = subprocess.run(
            [_V4L2_CTL, f"--device={device}", "--info"],
            capture_output=True, text=True, timeout=5, check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None, None, None
    def _grab(label: str) -> str | None:
        m = re.search(rf"{label}\s*:\s*(.+)", out)
        return m.group(1).strip() if m else None
    return _grab("Card type"), _grab("Driver name"), _grab("Bus info")


_FMT_RE = re.compile(r"\[\d+\]:\s*'(\w+)'")
_SIZE_RE = re.compile(r"Size: Discrete (\d+)x(\d+)")
_INTERVAL_RE = re.compile(r"Interval: Discrete [\d.]+s \(([\d.]+) fps\)")


def list_camera_formats(device: str) -> list[dict]:
    """Parse ``v4l2-ctl --list-formats-ext`` into structured entries."""
    if not _V4L2_CTL:
        return []
    try:
        out = subprocess.run(
            [_V4L2_CTL, f"--device={device}", "--list-formats-ext"],
            capture_output=True, text=True, timeout=5, check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return []

    formats: list[dict] = []
    fourcc: str | None = None
    size: tuple[int, int] | None = None
    for line in out.splitlines():
        m = _FMT_RE.search(line)
        if m:
            fourcc = m.group(1)
            continue
        m = _SIZE_RE.search(line)
        if m:
            size = (int(m.group(1)), int(m.group(2)))
            continue
        m = _INTERVAL_RE.search(line)
        if m and fourcc and size:
            formats.append({
                "fourcc": fourcc,
                "width": size[0],
                "height": size[1],
                "fps": float(m.group(1)),
            })
    return formats


def list_video_capture_devices(probe_formats: bool = True) -> list[CameraDeviceInfo]:
    """Every V4L2 node that actually reports a video-capture capability.

    A single UVC camera exposes several ``/dev/video*`` nodes (capture plus
    metadata); only nodes advertising a capture format are returned.
    """
    by_id = _link_map("/dev/v4l/by-id/*")
    cameras: list[CameraDeviceInfo] = []

    for node in sorted(glob.glob("/dev/video*"),
                       key=lambda p: int(re.sub(r"\D", "", p) or 0)):
        formats = list_camera_formats(node) if probe_formats else []
        if probe_formats and not formats:
            continue  # metadata-only or non-capture node
        card, driver, bus_info = _v4l2_capabilities(node)
        props = _udev_properties(node)
        cameras.append(
            CameraDeviceInfo(
                device=node,
                card=card,
                driver=driver,
                bus_info=bus_info,
                vendor_id=props.get("ID_VENDOR_ID"),
                product_id=props.get("ID_MODEL_ID"),
                serial=props.get("ID_SERIAL_SHORT"),
                by_id=by_id.get(os.path.realpath(node)),
                formats=formats,
            )
        )
    return cameras


class DeviceNotFound(RuntimeError):
    """Raised when a required device cannot be identified unambiguously."""


def _forced_device(matcher: DeviceMatcher) -> str | None:
    """An explicit ``device:`` in the config always wins -- it is deliberate."""
    if matcher.device and os.path.exists(matcher.device):
        return matcher.device
    return None


def _udev_candidate(matcher: DeviceMatcher, udev_link: str, devices, is_camera: bool):
    """Use the udev symlink only if it satisfies the configured matcher.

    The symlink is normally created by our own rules from the same VID/PID the
    config carries, so the two agree.  But if the rules and the config ever
    diverge -- someone edits one and not the other -- taking the symlink on
    faith would silently open the wrong device.  Verifying it here means a
    divergence falls through to identity matching instead of misidentifying.
    """
    if not os.path.exists(udev_link):
        return None
    real = os.path.realpath(udev_link)
    info = next((d for d in devices if os.path.realpath(d.device) == real), None)
    if matcher.is_empty():
        return udev_link, info          # nothing to check against
    if info is None:
        # The link exists but the device is not in our inventory; trusting it
        # blindly is exactly what this function is here to avoid.
        return None
    match = matcher.matches_camera if is_camera else matcher.matches_serial
    if not match(info):
        return None
    # Uniqueness still has to hold.  udev binds the symlink to ONE device even
    # when several match its rule -- plug a second CH340 in and both get
    # SYMLINK+="oasis_imu", with the winner chosen non-deterministically.
    # Accepting the link without this check would bypass the ambiguity guard
    # exactly when the udev rules are installed.
    if sum(1 for d in devices if match(d)) != 1:
        return None
    return udev_link, info


def resolve_serial(
    matcher: DeviceMatcher,
    udev_link: str,
    candidates: Iterable[SerialDeviceInfo] | None = None,
) -> tuple[str, SerialDeviceInfo | None]:
    """Resolve one serial device to an openable path.

    Precedence: explicit config override, then the udev stable symlink, then
    USB identity matching.  Raises :class:`DeviceNotFound` when nothing matches
    or when the matcher is ambiguous.
    """
    devices = list(candidates) if candidates is not None else list_serial_devices()

    forced = _forced_device(matcher)
    if forced:
        real = os.path.realpath(forced)
        info = next((d for d in devices if os.path.realpath(d.device) == real), None)
        return forced, info

    via_udev = _udev_candidate(matcher, udev_link, devices, is_camera=False)
    if via_udev:
        return via_udev

    if matcher.is_empty():
        raise DeviceNotFound(
            "no matcher configured; refusing to guess a serial device"
        )

    matches = [d for d in devices if matcher.matches_serial(d)]
    if not matches:
        seen = ", ".join(f"{d.device}({d.vid_pid})" for d in devices) or "none"
        raise DeviceNotFound(f"no serial device matched {matcher}; present: {seen}")
    if len(matches) > 1:
        seen = ", ".join(d.stable_path for d in matches)
        raise DeviceNotFound(
            f"matcher {matcher} is ambiguous across {len(matches)} devices: {seen}"
        )
    return matches[0].stable_path, matches[0]


def resolve_camera(
    matcher: DeviceMatcher,
    udev_link: str = UDEV_CAMERA,
    candidates: Iterable[CameraDeviceInfo] | None = None,
) -> tuple[str, CameraDeviceInfo | None]:
    """Resolve the capture camera the same way :func:`resolve_serial` does."""
    cameras = list(candidates) if candidates is not None else list_video_capture_devices()

    forced = _forced_device(matcher)
    if forced:
        real = os.path.realpath(forced)
        info = next((c for c in cameras if os.path.realpath(c.device) == real), None)
        return forced, info

    via_udev = _udev_candidate(matcher, udev_link, cameras, is_camera=True)
    if via_udev:
        return via_udev

    if matcher.is_empty():
        raise DeviceNotFound("no matcher configured; refusing to guess a camera")

    matches = [c for c in cameras if matcher.matches_camera(c)]
    if not matches:
        seen = ", ".join(f"{c.device}({c.card})" for c in cameras) or "none"
        raise DeviceNotFound(f"no camera matched {matcher}; present: {seen}")
    if len(matches) > 1:
        seen = ", ".join(c.device for c in matches)
        raise DeviceNotFound(f"matcher {matcher} is ambiguous: {seen}")
    return matches[0].device, matches[0]


def snapshot() -> dict:
    """Full device inventory, embedded verbatim into session metadata."""
    return {
        "serial": [d.to_dict() for d in list_serial_devices()],
        "cameras": [c.to_dict() for c in list_video_capture_devices()],
        "udev_links": {
            name: (os.path.realpath(path) if os.path.exists(path) else None)
            for name, path in (
                ("oasis_gps", UDEV_GPS),
                ("oasis_imu", UDEV_IMU),
                ("oasis_camera", UDEV_CAMERA),
            )
        },
    }


if __name__ == "__main__":  # pragma: no cover - operator convenience
    print(json.dumps(snapshot(), indent=2))
