#!/usr/bin/env python3
"""Build and flash the ESP32-C6, optionally overriding its BLE address.

By default the firmware derives its BLE address from the chip's factory MAC
(see esp/src/bin/main.rs), so every board is unique with no configuration -
just `pixi run esp-upload`. Pass --ble-address to compile in a specific
static-random address instead; it applies to this build only.

Read a board's current address back with `pixi run esp-address`.

  pixi run esp-upload                        # eFuse-derived, unique per board
  pixi run esp-upload --ble-address FF:...   # force a specific address
"""

import argparse
import os
import subprocess
from pathlib import Path

ESP_DIR = Path(__file__).resolve().parent.parent / "esp"


def normalize(addr: str) -> str:
    """Uppercase, validate a 6-octet colon MAC, or exit with a message."""
    octets = addr.strip().split(":")
    if len(octets) != 6 or any(len(o) != 2 for o in octets):
        raise SystemExit(f"bad BLE address {addr!r}: want six colon-separated hex octets")
    try:
        first = int(octets[0], 16)
        for o in octets:
            int(o, 16)
    except ValueError:
        raise SystemExit(f"bad BLE address {addr!r}: non-hex octet")
    if first & 0xC0 != 0xC0:
        raise SystemExit(
            f"bad BLE address {addr!r}: not static-random "
            "(first octet must be 0xC0-0xFF)"
        )
    return ":".join(o.upper() for o in octets)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ble-address", metavar="FF:C6:A1:53:50:47",
                    help="compile in this static-random address instead of the "
                         "eFuse-derived default")
    args = ap.parse_args()

    env = os.environ.copy()
    if args.ble_address is not None:
        addr = normalize(args.ble_address)
        env["BLE_ADDRESS"] = addr
        print(f"esp-upload: forcing BLE address {addr}")
    else:
        print("esp-upload: BLE address derived from the chip's eFuse MAC")

    # Inherit stdio so espflash's interactive monitor (see
    # esp/.cargo/config.toml) keeps its terminal handling and renders cleanly.
    try:
        return subprocess.run(["cargo", "run", "--release"], cwd=ESP_DIR, env=env).returncode
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
