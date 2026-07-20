#!/usr/bin/env python3
"""Push a radio config to the WIO-E5 through the ESP32-C6's USB port.

Sets this node's address without editing a file by hand:

    pixi run wio-config --address 3

Other keys go through --set, repeatably:

    pixi run wio-config --address 3 --set role=tx_only --set interval_s=30

The board applies the config immediately and rewrites its SD copy, so the
change survives a reboot; nothing has to be reflashed.

IMPORTANT - this sends a whole file, not a patch. The firmware parses a
config starting from its own defaults, so any key absent from what is sent
reverts to its default rather than keeping the board's current value. There
is no way to read a config back off the board, so the file this tool starts
from is the whole truth about the resulting settings.

That file is RADIO.example.toml by default, which a test in the proto crate
pins to the firmware defaults - so with no --set the board ends up on stock
settings plus the address given. If the board is running tuned radio
settings, pass the file holding them with --file and edit that instead:

    pixi run wio-config --file mynet.toml --address 3

--file also takes the SD card's own RADIO.CFG, which is the same format.
"""

import argparse
import sys
from pathlib import Path

import wio_link as link

DEFAULT_CONFIG = link.ROOT / "RADIO.example.toml"

# The parser on the WIO accepts 1-255; 0 is reserved to mark a packet as not
# one of ours (see proto/src/lora.rs).
ADDRESS_MIN, ADDRESS_MAX = 1, 255


def set_key(text: str, key: str, value: str) -> tuple[str, bool]:
    """Return `text` with `key` set to `value`, and whether it was already there.

    Matching is on the whole key before the '=', so `address` does not also
    hit the `address_description` line that documents it. The quoting style
    of the value being replaced is kept, which keeps a string value quoted
    for editors that read the file as real TOML (the firmware's own parser
    accepts it either way).
    """
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.split("=", 1)[0].strip() != key:
            continue
        old = line.split("=", 1)[1].strip()
        quoted = len(old) >= 2 and old[0] == old[-1] and old[0] in "\"'"
        if quoted and not (value[:1] in "\"'"):
            value = f'"{value}"'
        lines[i] = f"{key} = {value}"
        return "\n".join(lines) + "\n", True
    # Not in the file: append it. A key is valid under any section header,
    # so the end of the file is as good a place as any.
    return text.rstrip("\n") + f"\n\n{key} = {value}\n", False


def parse_set(arg: str) -> tuple[str, str]:
    key, sep, value = arg.partition("=")
    if not sep or not key.strip() or not value.strip():
        raise argparse.ArgumentTypeError(f"--set wants key=value, got {arg!r}")
    return key.strip(), value.strip()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--address", type=int, help=f"node address, {ADDRESS_MIN}-{ADDRESS_MAX}")
    ap.add_argument(
        "--set",
        type=parse_set,
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="set any other config key (repeatable), e.g. --set role=rx_only",
    )
    ap.add_argument(
        "--file",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"config to start from and edit (default: {DEFAULT_CONFIG})",
    )
    ap.add_argument("--port", help="serial port (auto-detected if omitted)")
    ap.add_argument(
        "--save",
        type=Path,
        help="also write the edited config here, to keep a record of what was sent",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="print the config that would be sent and exit without touching the board",
    )
    args = ap.parse_args()

    if args.address is None and not args.set:
        ap.error("nothing to change: pass --address and/or --set KEY=VALUE")
    if args.address is not None and not ADDRESS_MIN <= args.address <= ADDRESS_MAX:
        ap.error(f"--address {args.address} out of range {ADDRESS_MIN}-{ADDRESS_MAX}")

    if not args.file.is_file():
        sys.exit(f"config file not found: {args.file}")
    text = args.file.read_text()

    edits = ([("address", str(args.address))] if args.address is not None else []) + args.set
    unknown = []
    for key, value in edits:
        text, existed = set_key(text, key, value)
        print(f"  {key} = {value}" + ("" if existed else "   <- not in the file, appended"))
        if not existed:
            unknown.append(key)
    # The firmware ignores keys it does not recognize, so a misspelled one is
    # accepted and then does nothing. Every real key appears in the reference
    # file, so "not in the file" is the only warning available for a typo.
    if unknown and args.file == DEFAULT_CONFIG:
        print(f"warning: {', '.join(unknown)} not in the reference config. "
              "The firmware ignores keys it does not know, so a misspelling "
              "here is accepted and has no effect.")

    data = text.encode()
    if len(data) > 0xFFFFFFFF:
        sys.exit("config too large")

    if args.save:
        args.save.write_text(text)
        print(f"saved edited config to {args.save}")

    if args.dry_run:
        print(f"--- would send {len(data)} bytes (from {args.file}) ---")
        print(text, end="")
        return 0

    print(f"sending {len(data)} bytes based on {args.file}")
    ser = link.open_port(args.port)
    if not link.ping(ser):
        sys.exit("no PING reply - is the ESP running wio-e5-gps firmware?")
    print("ESP link alive")

    try:
        link.send_bulk(ser, link.KIND_TOML, data, version=0)
    except (TimeoutError, RuntimeError) as e:
        # The WIO parses the file at the end of the transfer, so a value it
        # rejects surfaces here rather than at begin - and looks identical to
        # a link fault unless it is spelled out.
        sys.exit(f"\nconfig push failed: {e}\n"
                 "A WIO NAK at this stage usually means the file itself was "
                 "rejected: a value out of range or a string that is not one "
                 "of the choices. Check it with --dry-run.")

    # The WIO logs "config applied, node N" once it has parsed and adopted
    # the file, which is the only read-back there is: an ack proves the
    # bytes arrived, this proves which address is now live.
    applied = link.read_console(ser, "config applied", timeout=3.0)
    if applied:
        print(applied)
    else:
        print("config accepted (no 'config applied' line seen; it may have "
              "scrolled past - the transfer itself was acked)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
