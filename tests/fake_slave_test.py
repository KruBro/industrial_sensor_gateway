#!/usr/bin/env python3
"""Run the master against a fake Modbus slave on a pseudo-terminal.

The fake slave answers every request according to a "mode" and counts the
requests it receives, so retry behaviour can be checked exactly.

Usage: python3 tests/fake_slave_test.py [master_binary] [master_binary_with_byte_delay]
Exit code 0 means every case behaved as expected.
"""
import os
import pty
import select
import subprocess
import sys
import time

BIN = sys.argv[1] if len(sys.argv) > 1 else "build/master"
BIN_DELAY = sys.argv[2] if len(sys.argv) > 2 else "build/master_delay"

VALUES = [3260, 3550, 450, 1, 2, 3, 4, 5, 6, 7]


def crc(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def frame(data):
    c = crc(data)
    return bytes(data) + bytes([c & 0xFF, c >> 8])


def ok_reply(req):
    addr = req[0]
    n = (req[4] << 8) | req[5]
    body = bytes([addr, 3, 2 * n])
    for i in range(n):
        v = VALUES[i] if i < len(VALUES) else 0
        body += bytes([v >> 8, v & 0xFF])
    return frame(body)


def bad_crc(reply):
    r = bytearray(reply)
    r[4 if len(r) > 5 else 3] ^= 0x01
    return bytes(r)


def make_reply(mode, n, req):
    """n is the 1-based index of this request. Return bytes, or None to stay silent."""
    addr = req[0]
    if mode in ("ok", "split"):
        return ok_reply(req)
    if mode == "exc":
        return frame([addr, 0x83, 3])
    if mode == "excbadcrc":
        return bad_crc(frame([addr, 0x83, 3]))
    if mode == "badcrc":
        return bad_crc(ok_reply(req))
    if mode == "wrongaddr":
        return frame([2, 3, 6, 0x0C, 0xBC, 0x0D, 0xDE, 0x01, 0xC2])
    if mode == "badcount":
        return frame([addr, 3, 4, 0x0C, 0xBC, 0x0D, 0xDE])
    if mode == "wrongfunc":
        return frame([addr, 4, 6, 0x0C, 0xBC, 0x0D, 0xDE, 0x01, 0xC2])
    if mode == "partial":
        return ok_reply(req)[:8]
    if mode == "first_lost":                     # request 1 ignored, the rest answered
        return None if n == 1 else ok_reply(req)
    if mode == "first_badcrc":                   # reply 1 damaged, the rest good
        return bad_crc(ok_reply(req)) if n == 1 else ok_reply(req)
    if mode == "odd_lost":                       # requests 1, 3, 5 ... ignored
        return None if n % 2 == 1 else ok_reply(req)
    return None                                  # "silent"


def run(binary, mode, args=(), port=None):
    master_fd, slave_fd = pty.openpty()          # keep slave_fd open: no hang-up
    name = port or os.ttyname(slave_fd)
    start = time.time()
    proc = subprocess.Popen([binary, name, *map(str, args)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    requests = 0
    if port is None:
        buf = b""
        while proc.poll() is None:
            ready, _, _ = select.select([master_fd], [], [], 0.05)
            if not ready:
                continue
            buf += os.read(master_fd, 64)
            while len(buf) >= 8:
                req, buf = buf[:8], buf[8:]
                requests += 1
                reply = make_reply(mode, requests, req)
                if reply is None:
                    continue
                if mode == "split":
                    for i in range(0, len(reply), 3):
                        os.write(master_fd, reply[i:i + 3])
                        time.sleep(0.15)
                else:
                    os.write(master_fd, reply)
    out, err = proc.communicate(timeout=30)
    return proc.returncode, out.strip(), err.strip(), time.time() - start, requests


NORMAL = "P=326.0 bar  F=355.0 L/min  T=45.0 C"

# title, binary, mode, args, port, exit code, texts that must appear, requests seen, max seconds
CASES = [
    # --- one read, with retries -------------------------------------------------
    ("normal reply (3 registers)",      BIN, "ok",        (),            None, 0, [NORMAL], 1, 1.0),
    ("reply arrives in 3-byte pieces",  BIN, "split",     (),            None, 0, [NORMAL], 1, 2.0),
    ("1 register",                      BIN, "ok",        (1,),          None, 0, ["reg[0] = 3260"], 1, 1.0),
    ("10 registers",                    BIN, "ok",        (10,),         None, 0, ["reg[9] = 7"], 1, 1.0),
    ("exception: no retry, fast",       BIN, "exc",       (11,),         None, 1, ["code 3 (illegal data value)"], 1, 1.0),
    ("first request lost -> retry",     BIN, "first_lost",   (),         None, 0, [NORMAL, "attempt 1/3: no reply (timeout)", "succeeded on attempt 2 of 3"], 2, 5.0),
    ("first reply bad CRC -> retry",    BIN, "first_badcrc", (),         None, 0, [NORMAL, "attempt 1/3: bad CRC"], 2, 1.0),
    ("bad CRC every time: 3 attempts",  BIN, "badcrc",    (),            None, 1, ["ERROR: bad CRC", "attempt 3/3: bad CRC"], 3, 2.0),
    ("exception with bad CRC",          BIN, "excbadcrc", (),            None, 1, ["ERROR: bad CRC"], 3, 2.0),
    ("wrong address in reply",          BIN, "wrongaddr", (),            None, 1, ["wrong slave address"], 3, 2.0),
    ("byte count mismatch",             BIN, "badcount",  (),            None, 1, ["byte count does not match"], 3, 2.0),
    ("wrong function code in reply",    BIN, "wrongfunc", (),            None, 1, ["unexpected function code"], 3, 2.0),
    ("partial reply every time",        BIN, "partial",   (),            None, 1, ["reply incomplete"], 3, 9.0),
    ("slave never answers",             BIN, "silent",    (),            None, 1, ["ERROR: no reply (timeout)", "attempt 3/3"], 3, 9.0),
    ("20 ms per-byte send delay",       BIN_DELAY, "ok",  (),            None, 0, [NORMAL], 1, 2.0),
    # --- counting mode ---------------------------------------------------------
    ("count: 5 good runs",              BIN, "ok",        (3, 1, 5),     None, 0, ["runs=5  ok_first_try=5  ok_after_retry=0  failed=0  attempts=5"], 5, 3.0),
    ("count: first try always lost",    BIN, "odd_lost",  (3, 1, 3),     None, 0, ["runs=3  ok_first_try=0  ok_after_retry=3  failed=0  attempts=6"], 6, 10.0),
    ("count: exceptions, 1 try each",   BIN, "exc",       (11, 1, 3),    None, 1, ["runs=3  ok_first_try=0  ok_after_retry=0  failed=3  attempts=3", "failures:  slave returned an exception=3"], 3, 2.0),
    ("count: bad CRC, failure breakdown", BIN, "badcrc",  (3, 1, 2),     None, 1, ["runs=2  ok_first_try=0  ok_after_retry=0  failed=2  attempts=6", "failures:  bad CRC=2"], 6, 3.0),
    # --- argument and port errors ----------------------------------------------
    ("port does not exist",             BIN, "ok",        (),            "/dev/does_not_exist", 1, ["open:"], 0, 1.0),
    ("count 0 rejected",                BIN, "ok",        (0,),          "/dev/null", 1, ["usage:"], 0, 1.0),
    ("runs 0 rejected",                 BIN, "ok",        (3, 1, 0),     "/dev/null", 1, ["usage:"], 0, 1.0),
    ("runs 1001 rejected",              BIN, "ok",        (3, 1, 1001),  "/dev/null", 1, ["usage:"], 0, 1.0),
]

failed = 0
for title, binary, mode, args, port, want_rc, texts, want_req, max_s in CASES:
    rc, out, err, secs, requests = run(binary, mode, args, port)
    text = out + "\n" + err
    problems = []
    if rc != want_rc:
        problems.append(f"exit {rc}, expected {want_rc}")
    for t in texts:
        if t not in text:
            problems.append(f"missing text {t!r}")
    if port is None and requests != want_req:
        problems.append(f"slave saw {requests} request(s), expected {want_req}")
    if secs > max_s:
        problems.append(f"took {secs:.1f}s, limit {max_s}s")
    print(f"{'FAIL' if problems else 'PASS'}  {title:36s} exit={rc} req={requests} {secs:4.1f}s")
    if problems:
        failed += 1
        for p in problems:
            print("      -", p)
        print(f"      stdout={out!r}\n      stderr={err!r}")

print(f"\n{len(CASES) - failed} passed, {failed} failed")
sys.exit(1 if failed else 0)
