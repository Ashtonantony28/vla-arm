#!/usr/bin/env python3
"""End-to-end test of the fw_sim CLI over pipes, plus a ctypes cross-check of libarmfw.so against
an independent pure-Python implementation of the framing and CRC (spec/protocol.md).

usage: fw_sim_pipe_test.py <fw_sim> [<libarmfw.so> | --no-lib]
"""
import ctypes
import os
import random
import select
import struct
import subprocess
import sys
import time


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(typ: int, seq: int, payload: bytes = b"") -> bytes:
    body = bytes([0x01, typ, seq & 0xFF, len(payload)]) + payload
    return b"\xA5\x5A" + body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xA5\x5A")
            if i < 0:
                del self.buf[: max(0, len(self.buf) - 1)]
                return out
            del self.buf[:i]
            if len(self.buf) < 6:
                return out
            ver, typ, seq, ln = self.buf[2:6]
            if ver != 1 or ln > 240:
                del self.buf[:1]
                continue
            if len(self.buf) < 8 + ln:
                return out
            body = bytes(self.buf[2 : 6 + ln])
            (crc,) = struct.unpack_from("<H", self.buf, 6 + ln)
            if crc != crc16(body):
                del self.buf[:1]
                continue
            out.append((typ, seq, bytes(self.buf[6 : 6 + ln])))
            del self.buf[: 8 + ln]


def parse_state(p: bytes):
    t_us, faults, mode, seq = struct.unpack_from("<IHBB", p, 0)
    joints = [struct.unpack_from("<hhhBB", p, 8 + 8 * j) for j in range(6)]
    return dict(t_us=t_us, faults=faults, mode=mode, seq=seq, pos=[j[0] for j in joints])


def check(cond, msg):
    if not cond:
        print("FAIL:", msg)
        sys.exit(1)
    print("ok  ", msg)


def lib_crosscheck(path):
    lib = ctypes.CDLL(path)
    lib.armfw_crc16.restype = ctypes.c_uint16
    lib.armfw_crc16.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
    lib.armfw_encode_frame.restype = ctypes.c_size_t
    lib.armfw_encode_frame.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_char_p, ctypes.c_size_t,
                                       ctypes.c_char_p, ctypes.c_size_t]
    check(lib.armfw_api_version() == 1, "lib api version 1")
    check(lib.armfw_crc16(b"123456789", 9) == 0x29B1 == crc16(b"123456789"), "CRC check value 0x29B1 (lib and Python)")
    rng = random.Random(1)
    out = ctypes.create_string_buffer(300)
    for _ in range(500):
        n = rng.randint(0, 240)
        pl = bytes(rng.randrange(256) for _ in range(n))
        typ, seq = rng.randrange(256), rng.randrange(256)
        assert lib.armfw_crc16(pl, n) == crc16(pl)
        m = lib.armfw_encode_frame(typ, seq, pl, n, out, 300)
        assert out.raw[:m] == frame(typ, seq, pl), "frame mismatch"
    check(True, "500 random frames: lib encode == Python encode")


def main():
    fw_sim = sys.argv[1]
    lib = sys.argv[2] if len(sys.argv) > 2 else "--no-lib"
    if lib != "--no-lib":
        lib_crosscheck(lib)

    proc = subprocess.Popen([fw_sim, "--log"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    parser = Parser()
    frames = []
    seq = 0

    def send(typ, payload=b""):
        nonlocal seq
        proc.stdin.write(frame(typ, seq, payload))
        proc.stdin.flush()
        seq += 1
        return seq - 1

    def pump(duration):
        end = time.monotonic() + duration
        while True:
            rem = end - time.monotonic()
            if rem <= 0:
                return
            r, _, _ = select.select([proc.stdout], [], [], rem)
            if r:
                data = os.read(proc.stdout.fileno(), 65536)
                if not data:
                    return
                for f in parser.feed(data):
                    frames.append((time.monotonic(), f))

    pump(0.05)
    s_ping = send(0x01)
    pump(0.1)
    pongs = [f for _, f in frames if f[0] == 0x81]
    check(len(pongs) == 1 and struct.unpack("<HIB", pongs[0][2])[2] == 1, "PING -> PONG (proto_ver 1)")
    check(pongs[0][1] is not None and s_ping == 0, "seq bookkeeping")

    target = [400, -300, 250, -150, 700, 1100]
    s_tgt = send(0x02, struct.pack("<H6hB", 300, *target, 1))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.5:
        send(0x01)
        pump(0.05)
    acks = [f for _, f in frames if f[0] == 0x83 and f[2] == bytes([s_tgt, 0x02])]
    check(len(acks) == 1, "SET_TARGETS acked")
    states = [(t, parse_state(f[2])) for t, f in frames if f[0] == 0x82]
    # STATE rate measured on the board's own timestamps over the last second
    last_t = states[-1][1]["t_us"]
    in_last_s = [s for _, s in states if last_t - s["t_us"] < 1_000_000]
    check(198 <= len(in_last_s) <= 202, f"STATE at 200 Hz on board time ({len(in_last_s)} in 1 s)")
    st = states[-1][1]
    check(st["mode"] == 1 and st["faults"] & 0x7F == 0, f"ACTIVE without faults (mode {st['mode']}, faults {st['faults']:#x})")
    err = max(abs(a - b) for a, b in zip(st["pos"], target))
    check(err <= 2, f"joints converged to target (max error {err} mrad)")

    # go silent: HOLD then WATCHDOG fault
    pump(0.7)
    st = [parse_state(f[2]) for _, f in frames if f[0] == 0x82][-1]
    check(st["mode"] == 3 and st["faults"] & 0x04, f"host silence -> FAULT/WATCHDOG (mode {st['mode']}, faults {st['faults']:#x})")

    proc.stdin.close()
    try:
        rc = proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()
        check(False, "fw_sim exits on stdin EOF")
    err_txt = proc.stderr.read().decode()
    print("    " + err_txt.strip())
    check(rc == 0, "fw_sim exited cleanly on EOF")
    print("all fw_sim pipe checks passed")


if __name__ == "__main__":
    main()
