#!/usr/bin/env python3
"""Interactive raw CDC client. Exit sends an immediate brake command."""
import argparse
import os
import select
import sys
import termios
import time

parser = argparse.ArgumentParser()
parser.add_argument("port", help="motor_terminal CDC device, e.g. /dev/ttyACM0")
args = parser.parse_args()
fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
saved = termios.tcgetattr(fd)
raw = termios.tcgetattr(fd)
raw[0] = raw[1] = raw[3] = 0
raw[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
raw[4] = raw[5] = termios.B115200
raw[6][termios.VMIN] = 0
raw[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, raw)
def send(data):
    deadline = time.monotonic() + 3
    while data:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError("serial write timed out")
        try:
            written = os.write(fd, data)
            data = data[written:]
        except BlockingIOError:
            continue

try:
    send(b"help\n")
    while True:
        ready, _, _ = select.select([fd, sys.stdin], [], [])
        if fd in ready:
            data = os.read(fd, 4096)
            if not data:
                break
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()
        if sys.stdin in ready:
            line = sys.stdin.readline()
            if not line or line.strip() == "quit":
                break
            send(line.encode())
except KeyboardInterrupt:
    pass
finally:
    try:
        send(b"!\n")
        termios.tcdrain(fd)
        termios.tcsetattr(fd, termios.TCSANOW, saved)
    except OSError:
        pass
    os.close(fd)
