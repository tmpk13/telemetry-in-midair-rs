#!/usr/bin/env python3
"""Upload a WIO-E5 firmware image through the ESP32-C6's USB Serial/JTAG port.

This tool streams a raw application image (objcopy of the wio-e5-gps ELF) to
the ESP over the bulk protocol in wio_link.py, which forwards it over the
UART link into the WIO's DFU partition; on a verified CRC the WIO reboots and
the swap bootloader installs it.

Everything is automatic - it builds the image (cargo objcopy in ../wio),
auto-detects the ESP port and uploads:

    pixi run wio-upload

Use --no-build to upload the existing image as-is, or --file / --port to
point elsewhere (an explicit --file is never rebuilt).

To change settings rather than firmware, see wio_config.py.
"""

import argparse
import subprocess
import sys
import zlib
from pathlib import Path

import wio_link as link

# Canonical WIO image: the objcopy output built in ../wio, resolved relative
# to this script so `pixi run wio-upload` works from any CWD.
#   cd wio && cargo objcopy --release -- -O binary wio-e5-gps.bin
DEFAULT_IMAGE = link.ROOT / "wio" / "wio-e5-gps.bin"

# The WIO ACTIVE partition size.
MAX_FW_SIZE = 56 * 2048


def build_image() -> None:
    """Build the WIO image with `cargo objcopy` in the wio/ crate."""
    wio_dir = DEFAULT_IMAGE.parent
    cmd = ["cargo", "objcopy", "--release", "--", "-O", "binary", DEFAULT_IMAGE.name]
    print(f"building image: {' '.join(cmd)} (in {wio_dir})")
    try:
        subprocess.run(cmd, cwd=wio_dir, check=True)
    except FileNotFoundError:
        sys.exit("cargo not found on PATH - build manually or pass --no-build")
    except subprocess.CalledProcessError as e:
        sys.exit(f"build failed (exit {e.returncode})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--file",
        type=Path,
        default=None,
        help=f"WIO firmware .bin to send (default: build {DEFAULT_IMAGE})",
    )
    ap.add_argument(
        "--no-build",
        action="store_true",
        help="upload the existing default image instead of rebuilding it",
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument("--version", type=int, default=1, help="firmware version stamp (0-65535)")
    args = ap.parse_args()

    # An explicit --file is used as-is; the default image is (re)built first
    # unless --no-build.
    if args.file is None:
        args.file = DEFAULT_IMAGE
        if not args.no_build:
            build_image()

    if not args.file.is_file():
        sys.exit(
            f"firmware image not found: {args.file}\n"
            "build it first: cd wio && cargo objcopy --release -- -O binary wio-e5-gps.bin"
        )
    print(f"image file: {args.file}")
    image = args.file.read_bytes()
    total = len(image)
    if total == 0 or total > MAX_FW_SIZE:
        sys.exit(f"image size {total} out of range (1..{MAX_FW_SIZE})")
    crc = zlib.crc32(image) & 0xFFFFFFFF
    print(f"image: {total} bytes, crc32 {crc:08x}, version {args.version}")

    ser = link.open_port(args.port)

    if not link.ping(ser):
        sys.exit("no PING reply - is the ESP running wio-e5-gps firmware?")
    print("ESP link alive")

    try:
        link.send_bulk(
            ser,
            link.KIND_FIRMWARE,
            image,
            version=args.version,
            hint="\nfor the first/recovery flash use SWD: cd wio && cargo run --release",
        )
    except (TimeoutError, RuntimeError) as e:
        sys.exit(f"\nupload failed: {e}")

    print("image verified; WIO rebooting into swap bootloader")
    return 0


if __name__ == "__main__":
    sys.exit(main())
