#!/usr/bin/env python3
"""Read the BLE address of a connected ESP32-C6 over USB.

The firmware answers a USB info query at any time, so this works whenever a
board is plugged in - no need to catch the one line it prints at boot. With
addresses derived from each chip's eFuse MAC every board is distinct, and
this is how you find out which is which.

  pixi run esp-address            # auto-detect the ESP port
  pixi run esp-address --port /dev/ttyACM0
"""

import argparse

import wio_link


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    args = ap.parse_args()

    ser = wio_link.open_port(args.port)
    try:
        addr = wio_link.query_ble_address(ser)
    finally:
        ser.close()

    if addr is None:
        raise SystemExit("no reply from the ESP (is it running this firmware?)")
    print(addr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
