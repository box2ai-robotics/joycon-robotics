#!/usr/bin/env python3
"""
Ubuntu Nintendo controller HID smoke test.

This test talks to Nintendo HID reports directly through hidapi. It is useful
for checking Bluetooth/USB, udev permissions, hid-nintendo, joycond, and the
Python environment before running robot teleoperation code.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import hid
except ImportError:
    print(
        "Missing Python module 'hid'. Run script/ubuntu_24_04_setup_joycon_uv.sh first, "
        "or install hidapi in the active environment.",
        file=sys.stderr,
    )
    raise SystemExit(1)


NINTENDO_VENDOR_ID = 0x057E
JOYCON_L_PRODUCT_ID = 0x2006
JOYCON_R_PRODUCT_ID = 0x2007
PRO_CONTROLLER_PRODUCT_ID = 0x2009
SUPPORTED_PRODUCT_IDS = {
    JOYCON_L_PRODUCT_ID,
    JOYCON_R_PRODUCT_ID,
    PRO_CONTROLLER_PRODUCT_ID,
}
NEUTRAL_RUMBLE = [0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40]


@dataclass
class NintendoDeviceInfo:
    index: int
    path: bytes
    vendor_id: int
    product_id: int
    product_string: str
    manufacturer_string: str
    serial_number: str
    interface_number: int | None

    @property
    def controller_type(self) -> str:
        if self.product_id == JOYCON_L_PRODUCT_ID:
            return "left"
        if self.product_id == JOYCON_R_PRODUCT_ID:
            return "right"
        if self.product_id == PRO_CONTROLLER_PRODUCT_ID:
            return "pro"
        return "unknown"


def _text(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _path_as_bytes(value) -> bytes:
    if isinstance(value, bytes):
        return value
    return str(value).encode()


def enumerate_nintendo_devices() -> list[NintendoDeviceInfo]:
    devices: list[NintendoDeviceInfo] = []
    for raw in hid.enumerate(0, 0):
        vendor_id = int(raw.get("vendor_id") or 0)
        product_id = int(raw.get("product_id") or 0)
        if vendor_id != NINTENDO_VENDOR_ID or product_id not in SUPPORTED_PRODUCT_IDS:
            continue

        path = raw.get("path")
        if not path:
            continue

        devices.append(
            NintendoDeviceInfo(
                index=len(devices),
                path=_path_as_bytes(path),
                vendor_id=vendor_id,
                product_id=product_id,
                product_string=_text(raw.get("product_string")),
                manufacturer_string=_text(raw.get("manufacturer_string")),
                serial_number=_text(raw.get("serial_number") or raw.get("serial")),
                interface_number=raw.get("interface_number"),
            )
        )
    return devices


def choose_device(
    devices: list[NintendoDeviceInfo],
    controller: str,
    side: str,
    index: int | None,
) -> NintendoDeviceInfo:
    if index is not None:
        for device in devices:
            if device.index == index:
                return device
        raise SystemExit(f"No supported Nintendo controller with index {index}.")

    controller = controller.lower()
    if controller != "any":
        matched = [device for device in devices if device.controller_type == controller]
        if not matched:
            raise SystemExit(f"No {controller} controller found.")
        return matched[0]

    side = side.lower()
    if side != "any":
        matched = [device for device in devices if device.controller_type == side]
        if not matched:
            raise SystemExit(f"No {side} controller found.")
        return matched[0]

    if not devices:
        raise SystemExit("No supported Nintendo controller found.")
    return devices[0]


def normalize_report(data: Iterable[int]) -> list[int]:
    report = list(data)
    if report and report[0] == 0x00 and len(report) > 1 and report[1] in (0x21, 0x30, 0x3F):
        return report[1:]
    return report


def write_report(device: hid.device, packet: list[int]) -> None:
    sizes = [49, 64, len(packet)]
    last_error: Exception | None = None
    for size in dict.fromkeys(sizes):
        payload = packet[:size] + [0x00] * max(0, size - len(packet))
        try:
            device.write(bytes(payload))
            return
        except OSError as exc:
            last_error = exc
    raise OSError(f"write failed for report sizes {sizes}: {last_error}")


class NintendoHID:
    def __init__(self, info: NintendoDeviceInfo):
        self.info = info
        self.device = hid.device()
        self.packet_number = 0

    def open(self) -> None:
        self.device.open_path(self.info.path)
        self.device.set_nonblocking(0)

    def close(self) -> None:
        try:
            self.device.close()
        except OSError:
            pass

    def read(self, timeout_ms: int = 1000) -> list[int]:
        return normalize_report(self.device.read(64, timeout_ms=timeout_ms))

    def send_subcommand(self, subcommand: int, argument: list[int], wait_ack: bool = True) -> bool:
        packet = [0x01, self.packet_number & 0x0F]
        packet.extend(NEUTRAL_RUMBLE)
        packet.append(subcommand)
        packet.extend(argument)
        write_report(self.device, packet)
        self.packet_number = (self.packet_number + 1) & 0x0F

        if not wait_ack:
            time.sleep(0.03)
            return True

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            report = self.read(timeout_ms=80)
            if len(report) >= 15 and report[0] == 0x21 and report[14] == subcommand:
                return bool(report[13] & 0x80)
        return False

    def initialize(self) -> None:
        steps = [
            (0x03, [0x30], "set input report mode 0x30"),
            (0x40, [0x01], "enable IMU"),
            (0x48, [0x01], "enable vibration"),
        ]
        for subcommand, argument, label in steps:
            ok = self.send_subcommand(subcommand, argument, wait_ack=True)
            print(f"{label}: {'ACK' if ok else 'no ACK'}")
            time.sleep(0.08)


def parse_stick_at(report: list[int], offset: int) -> tuple[int, int, float, float]:
    raw_x = report[offset] | ((report[offset + 1] & 0x0F) << 8)
    raw_y = (report[offset + 1] >> 4) | (report[offset + 2] << 4)
    norm_x = max(-1.0, min(1.0, (raw_x - 2048) / 2048.0))
    norm_y = max(-1.0, min(1.0, (raw_y - 2048) / 2048.0))
    return raw_x, raw_y, norm_x, norm_y


def parse_stick(report: list[int], controller_type: str) -> tuple[int, int, float, float]:
    offset = 6 if controller_type == "left" else 9
    return parse_stick_at(report, offset)


def parse_buttons(report: list[int], controller_type: str) -> list[str]:
    right = report[3]
    shared = report[4]
    left = report[5]
    buttons: list[str] = []

    right_mapping = [
        ("Y", 0x01),
        ("X", 0x02),
        ("B", 0x04),
        ("A", 0x08),
        ("SR_R", 0x10),
        ("SL_R", 0x20),
        ("R", 0x40),
        ("ZR", 0x80),
    ]
    left_mapping = [
        ("DOWN", 0x01),
        ("UP", 0x02),
        ("RIGHT", 0x04),
        ("LEFT", 0x08),
        ("SR_L", 0x10),
        ("SL_L", 0x20),
        ("L", 0x40),
        ("ZL", 0x80),
    ]

    if controller_type in ("right", "pro"):
        buttons.extend(name for name, mask in right_mapping if right & mask)
        if shared & 0x02:
            buttons.append("+")
        if shared & 0x04:
            buttons.append("R_STICK")
        if shared & 0x10:
            buttons.append("HOME")

    if controller_type in ("left", "pro"):
        buttons.extend(name for name, mask in left_mapping if left & mask)
        if shared & 0x01:
            buttons.append("-")
        if shared & 0x08:
            buttons.append("L_STICK")
        if shared & 0x20:
            buttons.append("CAPTURE")
    return buttons


def parse_imu_sample(report: list[int], sample_index: int = 0) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    offset = 13 + sample_index * 12
    accel_raw = [
        int.from_bytes(bytes(report[offset + i : offset + i + 2]), "little", signed=True)
        for i in (0, 2, 4)
    ]
    gyro_raw = [
        int.from_bytes(bytes(report[offset + i : offset + i + 2]), "little", signed=True)
        for i in (6, 8, 10)
    ]
    accel_g = tuple(value / 4096.0 for value in accel_raw)
    gyro_dps = tuple(value / 13.371 for value in gyro_raw)
    return accel_g, gyro_dps


def run_test(controller: NintendoHID, duration: float, rate_hz: float) -> int:
    controller.initialize()

    print("\nReading input reports. Move sticks, press buttons, and rotate the controller.")
    print("Press Ctrl+C to stop.\n")

    start = time.monotonic()
    next_print = start
    reports = 0
    report30 = 0
    last_error = ""

    while time.monotonic() - start < duration:
        try:
            report = controller.read(timeout_ms=250)
        except OSError as exc:
            last_error = str(exc)
            continue

        reports += 1
        if len(report) < 49 or report[0] != 0x30:
            continue

        report30 += 1
        now = time.monotonic()
        if now < next_print:
            continue
        next_print = now + (1.0 / rate_hz)

        controller_type = controller.info.controller_type
        raw_x, raw_y, stick_x, stick_y = parse_stick(report, controller_type)
        buttons = parse_buttons(report, controller_type)
        accel_g, gyro_dps = parse_imu_sample(report, 0)
        roll = math.degrees(math.atan2(accel_g[1], -accel_g[2]))
        pitch = math.degrees(math.atan2(accel_g[0], math.sqrt(accel_g[1] ** 2 + accel_g[2] ** 2)))

        if controller_type == "pro":
            l_raw_x, l_raw_y, l_x, l_y = parse_stick_at(report, 6)
            r_raw_x, r_raw_y, r_x, r_y = parse_stick_at(report, 9)
            stick_text = (
                f"left_stick=({l_x:+.2f},{l_y:+.2f}) raw=({l_raw_x:4d},{l_raw_y:4d}) "
                f"right_stick=({r_x:+.2f},{r_y:+.2f}) raw=({r_raw_x:4d},{r_raw_y:4d})"
            )
        else:
            stick_text = f"stick=({stick_x:+.2f},{stick_y:+.2f}) raw=({raw_x:4d},{raw_y:4d})"

        print(
            f"report=0x30 count={report30:04d} "
            f"{stick_text} "
            f"buttons={buttons or '-'} "
            f"accel_g=({accel_g[0]:+.2f},{accel_g[1]:+.2f},{accel_g[2]:+.2f}) "
            f"gyro_dps=({gyro_dps[0]:+.1f},{gyro_dps[1]:+.1f},{gyro_dps[2]:+.1f}) "
            f"tilt=({roll:+.1f},{pitch:+.1f})"
        )

    print(f"\nSummary: total reports={reports}, input report 0x30={report30}")
    if report30 == 0:
        print("No 0x30 input reports were received.")
        if last_error:
            print(f"Last read error: {last_error}")
        return 2
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Test Nintendo controller HID access on Ubuntu.")
    parser.add_argument("--controller", choices=["right", "left", "pro", "any"], default="any")
    parser.add_argument("--side", choices=["right", "left", "any"], default="any", help="Compatibility alias for Joy-Con selection.")
    parser.add_argument("--index", type=int, help="Use a specific index from the detected controller list.")
    parser.add_argument("--duration", type=float, default=20.0, help="Seconds to read data.")
    parser.add_argument("--rate", type=float, default=5.0, help="Print rate in Hz.")
    parser.add_argument("--list", action="store_true", help="Only list detected supported controllers.")
    args = parser.parse_args()

    devices = enumerate_nintendo_devices()
    print("Detected supported Nintendo HID devices:")
    if not devices:
        print("  none")
    for device in devices:
        path_preview = device.path.decode("utf-8", errors="replace")
        print(
            f"  [{device.index}] controller={device.controller_type:5s} "
            f"vid=0x{device.vendor_id:04X} pid=0x{device.product_id:04X} "
            f"product={device.product_string!r} serial={device.serial_number!r} "
            f"iface={device.interface_number!r} path={path_preview}"
        )

    if args.list:
        return 0
    if not devices:
        print("\nPair/connect a Joy-Con or Nintendo Switch Pro Controller, then rerun this script.")
        return 1

    selected = choose_device(devices, args.controller, args.side, args.index)
    print(f"\nOpening [{selected.index}] {selected.controller_type} controller: {selected.product_string!r}")

    controller = NintendoHID(selected)
    try:
        controller.open()
        return run_test(controller, duration=args.duration, rate_hz=args.rate)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except OSError as exc:
        print(f"\nHID error: {exc}")
        print("Check that udev rules are loaded, the controller is connected, and your user can access hidraw.")
        print("You can retry after running: sudo udevadm control --reload-rules && sudo udevadm trigger")
        return 1
    finally:
        controller.close()


if __name__ == "__main__":
    sys.exit(main())
