#!/usr/bin/env python3
"""Upload a WIO-E5 firmware image through the ESP32-C6's USB Serial/JTAG port.

The ESP32-C6 firmware exposes the same bulk-transfer protocol it serves over
BLE on its USB console, framed with the midair-proto link framing (see
proto/src/link.rs, module `usb`). This tool streams a raw application image
(objcopy of the wio-e5-gps ELF) to the ESP, which forwards it over the UART
link into the WIO's DFU partition; on a verified CRC the WIO reboots and the
swap bootloader installs it.

Everything is automatic - it builds the image (cargo objcopy in ../wio),
auto-detects the ESP port and uploads:

    pixi run fw-upload

Use --no-build to upload the existing image as-is, or --file / --port to
point elsewhere (an explicit --file is never rebuilt).

The ESP console shares this port, so its text is interleaved with the reply
frames; the frame parser here resyncs past it by sync byte and CRC.
"""

import argparse
import subprocess
import sys
import time
import zlib
from pathlib import Path

import serial
from serial.tools import list_ports

# Canonical WIO image: the objcopy output built in ../wio, resolved relative
# to this script so `pixi run fw-upload` works from any CWD.
#   cd wio && cargo objcopy --release -- -O binary wio-e5-gps.bin
DEFAULT_IMAGE = Path(__file__).resolve().parent.parent / "wio" / "wio-e5-gps.bin"

# -- Wire protocol constants (mirror of proto/src/link.rs and ble.rs) --------

SYNC = 0xAA
MAX_PAYLOAD = 256

RESP_ACK = 0x81
RESP_NAK = 0x82

USB_PING = 0x50
USB_BULK = 0x51
USB_BULK_ACK = 0x52

OP_BEGIN = 0x01
OP_DATA = 0x02
OP_END = 0x03
OP_ABORT = 0x04

KIND_FIRMWARE = 2

ACK_ID_BULK = 0x20
ACK_OK = 0

# Data bytes per OP_DATA (link::DATA_CHUNK) and the WIO ACTIVE partition size.
DATA_CHUNK = 192
MAX_FW_SIZE = 56 * 2048

# Espressif USB vendor id, used to auto-detect the port.
ESPRESSIF_VID = 0x303A


def crc8(data: bytes) -> int:
    """CRC-8/ITU (poly 0x07, init 0) over the given bytes."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, payload: bytes) -> bytes:
    plen = len(payload)
    if plen > MAX_PAYLOAD:
        raise ValueError("payload too large")
    body = bytes([cmd]) + payload
    return bytes([SYNC, plen & 0xFF, (plen >> 8) & 0xFF]) + body + bytes([crc8(body)])


class FrameParser:
    """Byte-at-a-time parser matching proto::link::FrameParser."""

    def __init__(self):
        self.state = "sync"
        self.buf = bytearray()
        self.expected = 0

    def feed(self, byte: int):
        """Return (cmd, payload) on a complete, CRC-valid frame, else None."""
        if self.state == "sync":
            if byte == SYNC:
                self.state = "lenlo"
        elif self.state == "lenlo":
            self.expected = byte
            self.state = "lenhi"
        elif self.state == "lenhi":
            self.expected |= byte << 8
            if self.expected > MAX_PAYLOAD:
                self.state = "sync"
            else:
                self.expected += 1  # + cmd byte
                self.buf = bytearray()
                self.state = "data"
        elif self.state == "data":
            self.buf.append(byte)
            if len(self.buf) >= self.expected:
                self.state = "crc"
        elif self.state == "crc":
            self.state = "sync"
            if crc8(bytes(self.buf)) == byte:
                return self.buf[0], bytes(self.buf[1:])
        return None


def read_frame(ser: serial.Serial, wanted: set, timeout: float):
    """Read until a frame whose cmd is in `wanted` arrives, or timeout."""
    parser = FrameParser()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        for byte in chunk:
            got = parser.feed(byte)
            if got and got[0] in wanted:
                return got
    return None


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


def open_port(port: str | None) -> serial.Serial:
    if port is None:
        for p in list_ports.comports():
            if p.vid == ESPRESSIF_VID:
                port = p.device
                print(f"auto-detected ESP port {port} ({p.description})")
                break
    if port is None:
        sys.exit("no --port given and no Espressif USB serial port found")
    # USB CDC-ACM ignores the baud rate; the value is a placeholder.
    return serial.Serial(port, 115200, timeout=0.05)


def bulk_op(ser: serial.Serial, op_payload: bytes, timeout: float = 3.0):
    """Send one bulk op and return (status, next_seq) from the ESP's ack."""
    ser.reset_input_buffer()
    ser.write(build_frame(USB_BULK, op_payload))
    ser.flush()
    frame = read_frame(ser, {USB_BULK_ACK}, timeout)
    if frame is None:
        raise TimeoutError("no ack from ESP")
    _, payload = frame
    if len(payload) < 2 or payload[0] != ACK_ID_BULK:
        raise ValueError(f"unexpected ack payload {payload.hex()}")
    status = payload[1]
    next_seq = int.from_bytes(payload[2:6], "little") if len(payload) >= 6 else 0
    return status, next_seq


def ping(ser: serial.Serial) -> bool:
    ser.reset_input_buffer()
    ser.write(build_frame(USB_PING, b""))
    ser.flush()
    frame = read_frame(ser, {RESP_ACK}, 2.0)
    return frame is not None and frame[1][:1] == bytes([USB_PING])


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
    with open(args.file, "rb") as f:
        image = f.read()
    total = len(image)
    if total == 0 or total > MAX_FW_SIZE:
        sys.exit(f"image size {total} out of range (1..{MAX_FW_SIZE})")
    crc = zlib.crc32(image) & 0xFFFFFFFF
    print(f"image: {total} bytes, crc32 {crc:08x}, version {args.version}")

    ser = open_port(args.port)

    if not ping(ser):
        sys.exit("no PING reply - is the ESP running wio-e5-gps firmware?")
    print("ESP link alive")

    begin = bytes([OP_BEGIN, KIND_FIRMWARE]) + total.to_bytes(4, "little") \
        + crc.to_bytes(4, "little") + args.version.to_bytes(2, "little")
    status, _ = bulk_op(ser, begin, timeout=3.0)
    if status != ACK_OK:
        sys.exit(f"begin rejected (status {status})")
    print("transfer started")

    seq = 0
    sent = 0
    for off in range(0, total, DATA_CHUNK):
        chunk = image[off:off + DATA_CHUNK]
        op = bytes([OP_DATA]) + seq.to_bytes(2, "little") + chunk
        status, next_seq = bulk_op(ser, op, timeout=3.0)
        if status != ACK_OK:
            bulk_op(ser, bytes([OP_ABORT]))
            sys.exit(f"\nchunk seq {seq} rejected (status {status})")
        seq = next_seq & 0xFFFF
        sent += len(chunk)
        pct = 100 * sent // total
        print(f"\r  {sent}/{total} bytes ({pct}%)", end="", flush=True)
    print()

    status, _ = bulk_op(ser, bytes([OP_END]), timeout=8.0)
    if status != ACK_OK:
        sys.exit(f"end/verify failed (status {status})")
    print("image verified; WIO rebooting into swap bootloader")
    return 0


if __name__ == "__main__":
    sys.exit(main())
