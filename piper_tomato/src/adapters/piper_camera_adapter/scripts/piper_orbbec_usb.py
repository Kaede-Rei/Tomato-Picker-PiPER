#!/usr/bin/env python3
import argparse
import os
import re
import sys


def _usb_uid_from_video_device(video_device: str) -> str:
    video_name = os.path.basename(os.path.realpath(video_device))
    if not video_name.startswith("video"):
        return ""

    sys_device = f"/sys/class/video4linux/{video_name}/device"
    try:
        resolved = os.path.realpath(sys_device)
    except OSError:
        return ""

    matches = re.findall(r"([0-9]+-[0-9]+(?:\.[0-9]+)*)(?::[0-9.]+)?", resolved)
    return matches[-1] if matches else ""


def _usb_uid_from_com_alias(alias_name: str) -> str:
    match = re.fullmatch(r"com-([0-9][0-9.-]*)-video", alias_name)
    if not match:
        return ""

    token = match.group(1)
    if "-" not in token:
        token = f"1-{token}"

    sys_usb_devices = "/sys/bus/usb/devices"
    try:
        for device_name in os.listdir(sys_usb_devices):
            if ":" in device_name or not device_name.endswith(token):
                continue
            vendor_path = os.path.join(sys_usb_devices, device_name, "idVendor")
            try:
                with open(vendor_path, "r", encoding="utf-8") as vendor_file:
                    if vendor_file.read().strip() == "2bc5":
                        return device_name
            except OSError:
                pass
    except OSError:
        pass

    return ""


def resolve_usb_port_selector(selector: str) -> str:
    value = str(selector or "").strip()
    if not value:
        return ""

    video_match = re.fullmatch(r"(?:/dev/)?video([0-9]+)", value)
    if video_match:
        uid = _usb_uid_from_video_device(f"/dev/video{video_match.group(1)}")
        return uid or value

    if re.fullmatch(r"[0-9]+", value):
        uid = _usb_uid_from_video_device(f"/dev/video{value}")
        return uid or value

    if value.startswith("/dev/"):
        uid = _usb_uid_from_video_device(value)
        if uid:
            return uid
        alias_uid = _usb_uid_from_com_alias(os.path.basename(value))
        if alias_uid:
            return alias_uid

    return value


def _needs_strict_resolution(value: str) -> bool:
    return value.startswith("/dev/") or re.fullmatch(r"[0-9]+", value) is not None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Resolve Orbbec /dev/video*, video index, or /dev/com-*-video selectors to SDK USB UID."
    )
    parser.add_argument("selector")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="fail if a device-style selector cannot be resolved to a USB UID",
    )
    args = parser.parse_args()

    resolved = resolve_usb_port_selector(args.selector)
    if args.strict and _needs_strict_resolution(args.selector) and resolved == args.selector:
        print(
            f"failed to resolve Orbbec selector '{args.selector}' to USB UID",
            file=sys.stderr,
        )
        return 2

    print(resolved)
    return 0


if __name__ == "__main__":
    sys.exit(main())
