#!/usr/bin/env python3
"""Shared host-side transport for talking to the board over USB.

The ESP32-C6 firmware exposes the same bulk-transfer protocol it serves over
BLE on its USB Serial/JTAG console, framed with the midair-proto link framing
(see proto/src/link.rs, module `usb`). Both a firmware image and a TOML radio
config travel that path, differing only in the `kind` byte, so the framing,
port detection and retry logic live here and the tools on top stay short.

The ESP console shares this port, so its text is interleaved with the reply
frames; the frame parser resyncs past it by sync byte and CRC.
"""

import time
from pathlib import Path

import serial
from serial.tools import list_ports

# -- Wire protocol constants (mirror of proto/src/link.rs and ble.rs) --------

SYNC = 0xAA
MAX_PAYLOAD = 256

RESP_ACK = 0x81
RESP_NAK = 0x82

USB_PING = 0x50
USB_BULK = 0x51
USB_BULK_ACK = 0x52
USB_INFO = 0x53

OP_BEGIN = 0x01
OP_DATA = 0x02
OP_END = 0x03
OP_ABORT = 0x04

KIND_TOML = 1
KIND_FIRMWARE = 2

ACK_ID_BULK = 0x20
ACK_OK = 0

# Bulk ack status codes (gps-proto packet ACK_* + midair-proto ble ACK_*).
STATUS_NAMES = {
    0x00: "OK",
    0x01: "unknown id",
    0x02: "bad value",
    0x10: "WIO error (NAK from the WIO)",
    0x11: "WIO link timeout (ESP got no ack from the WIO)",
    0x12: "bad state (a transfer is already active?)",
}

# Data bytes per OP_DATA (link::DATA_CHUNK).
DATA_CHUNK = 192

# Espressif USB vendor id, used to auto-detect the port.
ESPRESSIF_VID = 0x303A

# How many times to retry a bulk op before giving up (transport hiccups and,
# for OP_END, WIO-side link timeouts).
ATTEMPTS = 10

# Repo root, resolved from this file so a task works from any CWD.
ROOT = Path(__file__).resolve().parent.parent


def status_str(status: int) -> str:
    return f"{status} ({STATUS_NAMES.get(status, 'unknown')})"


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
    """Read until a frame whose cmd is in `wanted` arrives, or timeout.

    Returns (frame_or_None, info). `info` describes what was seen while
    waiting - bytes read, any other frames, and a sample of console text -
    so a caller can explain *why* it is retrying instead of just "no ack".
    """
    parser = FrameParser()
    deadline = time.monotonic() + timeout
    nbytes = 0
    others: list[int] = []
    text = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        nbytes += len(chunk)
        for byte in chunk:
            got = parser.feed(byte)
            if got is not None:
                if got[0] in wanted:
                    return got, ""
                others.append(got[0])
            elif 32 <= byte < 127 and len(text) < 96:
                text.append(byte)
    info = f"{nbytes} B in {timeout:.0f}s"
    if others:
        info += " other frames " + ",".join(f"0x{c:02x}" for c in others)
    if text:
        info += f" console {bytes(text).decode('ascii', 'replace')!r}"
    return None, info


def read_console(ser: serial.Serial, match: str, timeout: float) -> str | None:
    """Watch the console for a line containing `match`, ignoring frames.

    The WIO's own status lines reach this port, so a tool can confirm what
    the board did rather than only that the transfer was acked.
    """
    deadline = time.monotonic() + timeout
    line = bytearray()
    while time.monotonic() < deadline:
        for byte in ser.read(64):
            if byte in (0x0A, 0x0D):
                text = bytes(line).decode("ascii", "replace")
                line.clear()
                if match in text:
                    return text.strip()
            elif 32 <= byte < 127:
                if len(line) < 200:
                    line.append(byte)
            else:
                # A frame byte mid-line: the line is not console text.
                line.clear()
    return None


def open_port(port: str | None) -> serial.Serial:
    if port is None:
        for p in list_ports.comports():
            if p.vid == ESPRESSIF_VID:
                port = p.device
                print(f"auto-detected ESP port {port} ({p.description})")
                break
    if port is None:
        raise SystemExit("no --port given and no Espressif USB serial port found")
    # USB CDC-ACM ignores the baud rate; the value is a placeholder.
    return serial.Serial(port, 115200, timeout=0.05)


def bulk_op(ser: serial.Serial, op_payload: bytes, timeout: float = 3.0):
    """Send one bulk op and return (status, next_seq) from the ESP's ack."""
    ser.reset_input_buffer()
    ser.write(build_frame(USB_BULK, op_payload))
    ser.flush()
    frame, info = read_frame(ser, {USB_BULK_ACK}, timeout)
    if frame is None:
        raise TimeoutError(f"no ack from ESP ({info})")
    _, payload = frame
    if len(payload) < 2 or payload[0] != ACK_ID_BULK:
        raise ValueError(f"unexpected ack payload {payload.hex()}")
    status = payload[1]
    next_seq = int.from_bytes(payload[2:6], "little") if len(payload) >= 6 else 0
    return status, next_seq


def bulk_op_retry(ser: serial.Serial, op_payload: bytes, timeout: float, label: str,
                  attempts: int = ATTEMPTS) -> tuple[int, int]:
    """`bulk_op` with retries on transport hiccups (a lost/garbled ack).

    Retrying is safe: the ESP and WIO both de-duplicate by sequence number,
    so re-sending the same frame either re-acks (already applied) or applies
    it now. A returned protocol status (incl. a NAK) is passed straight back
    to the caller; only transport failures (timeout / bad ack frame) retry.
    """
    last: Exception = TimeoutError(f"{label}: no attempts made")
    for attempt in range(1, attempts + 1):
        try:
            return bulk_op(ser, op_payload, timeout)
        except (TimeoutError, ValueError) as e:
            last = e
            if attempt >= attempts:
                break
            print(f"\n  {label}: {e}; retry {attempt}/{attempts - 1}", flush=True)
            ser.reset_input_buffer()
            time.sleep(0.1 * attempt)
    raise TimeoutError(f"{label}: gave up after {attempts} tries ({last})") from last


def send_end(ser: serial.Serial, attempts: int = ATTEMPTS) -> None:
    """Finalize the transfer (OP_END), robust to both a dropped ESP ack and a
    WIO-side link timeout. Returns on success; raises on a definitive failure.

    The end step erases/CRC-checks flash and the WIO reboots, so its ack can
    be lost more easily than a data ack - hence the extra WIO-timeout retry on
    top of the transport retry. The ESP keeps the transfer open on a WIO
    timeout, so re-sending OP_END is safe.
    """
    for attempt in range(1, attempts + 1):
        last = attempt >= attempts
        try:
            status, _ = bulk_op(ser, bytes([OP_END]), timeout=8.0)
        except (TimeoutError, ValueError) as e:
            if last:
                raise TimeoutError(f"end: no ESP reply after {attempts} tries ({e})") from e
            print(f"\n  end: {e}; retry {attempt}/{attempts - 1}", flush=True)
            ser.reset_input_buffer()
            time.sleep(0.2 * attempt)
            continue
        if status == ACK_OK:
            return
        # 0x11 = WIO link timeout: the round-trip did not complete; the ESP
        # kept the transfer open, so retrying is safe.
        if status == 0x11 and not last:
            print(f"\n  end: {status_str(status)}; retry {attempt}/{attempts - 1}", flush=True)
            time.sleep(0.3)
            continue
        # 0x12 = ESP has no active transfer. On a retry this means a previous
        # OP_END already finalized it (its ack was lost) - the work is
        # committed, so treat as success. On the first try it is a real error
        # (state vanished without finishing).
        if status == 0x12 and attempt > 1:
            print("\n  end: ESP reports the transfer already finalized; "
                  "treating as success")
            return
        raise RuntimeError(f"end/verify failed: status {status_str(status)}")
    raise TimeoutError(f"end: WIO never confirmed after {attempts} tries")


def ping(ser: serial.Serial) -> bool:
    ser.reset_input_buffer()
    ser.write(build_frame(USB_PING, b""))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, 2.0)
    return frame is not None and frame[1][:1] == bytes([USB_PING])


def query_ble_address(ser: serial.Serial, timeout: float = 2.0) -> str | None:
    """Ask the ESP for its BLE address on demand; return "FF:C6:..." or None.

    The reply is [USB_INFO, addr[0]..addr[5]] with the address most-
    significant octet first, so it prints directly.
    """
    ser.reset_input_buffer()
    ser.write(build_frame(USB_INFO, b""))
    ser.flush()
    frame, _ = read_frame(ser, {RESP_ACK}, timeout)
    if frame is None:
        return None
    payload = frame[1]
    if len(payload) < 7 or payload[0] != USB_INFO:
        return None
    return ":".join(f"{b:02X}" for b in payload[1:7])


def send_bulk(ser: serial.Serial, kind: int, data: bytes, version: int = 0,
              progress: bool = True, hint: str = "") -> None:
    """Run a whole begin/data/end bulk transfer, aborting on failure.

    Raises SystemExit with a diagnosis on a protocol rejection, or lets
    TimeoutError/RuntimeError out after a best-effort abort. `hint` is
    appended to the "WIO is not answering" diagnosis, for advice that only
    makes sense for one kind of transfer.
    """
    import zlib

    total = len(data)
    crc = zlib.crc32(data) & 0xFFFFFFFF
    begin = bytes([OP_BEGIN, kind]) + total.to_bytes(4, "little") \
        + crc.to_bytes(4, "little") + version.to_bytes(2, "little")
    try:
        status, _ = bulk_op_retry(ser, begin, timeout=3.0, label="begin")
        if status != ACK_OK:
            msg = f"begin rejected: status {status_str(status)}"
            if status in (0x10, 0x11):
                msg += (
                    "\nthe ESP could not get an ack from the WIO. Is the WIO running "
                    "working firmware, powered (ESP GPIO2 rail on) and not held in reset?"
                ) + hint
            raise SystemExit(msg)
        seq = 0
        sent = 0
        for off in range(0, total, DATA_CHUNK):
            chunk = data[off:off + DATA_CHUNK]
            op = bytes([OP_DATA]) + seq.to_bytes(2, "little") + chunk
            status, next_seq = bulk_op_retry(ser, op, timeout=3.0, label=f"chunk seq {seq}")
            if status != ACK_OK:
                raise SystemExit(f"\nchunk seq {seq} rejected: status {status_str(status)}")
            seq = next_seq & 0xFFFF
            sent += len(chunk)
            if progress:
                print(f"\r  {sent}/{total} bytes ({100 * sent // total}%)", end="", flush=True)
        if progress:
            print()
        send_end(ser)
    except (TimeoutError, RuntimeError, SystemExit):
        # Best-effort abort so the WIO/ESP do not sit waiting for the rest.
        try:
            bulk_op(ser, bytes([OP_ABORT]), timeout=1.0)
        except (TimeoutError, ValueError):
            pass
        raise
