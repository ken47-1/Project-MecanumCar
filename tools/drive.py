#!/usr/bin/env python3
"""
Keyboard driver for Project MecanumCar over Bluetooth serial.

Sends motion commands to the HC-05/HC-06 at 25 Hz and auto-releases
when the key is no longer held. Uses GetAsyncKeyState so press-and-hold
works without OS key-repeat delay.

Multiple keys held at once are sent together in one frame. The firmware
parser accumulates every character in one pass, so a multi-char send
becomes one combined MotionCommand.

Usage:
    python drive.py COM5
    python drive.py COM5 --baud 9600
    python drive.py COM5 --quiet

Keys (hold for continuous motion, combine freely):
    W A S D     forward, strafe left, backward, strafe right
    Q E Z C     diagonals (FL, FR, BL, BR)
    J L         spin CCW / CW
    SPACE       stop (overrides any motion key)

Combining two keys is equivalent to the diagonal char:
    W + A  ==  Q     (forward-left)
    W + D  ==  E     (forward-right)
    S + A  ==  Z     (backward-left)
    S + D  ==  C     (backward-right)

System (one-shot):
    0           manual mode
    1           autonomous mode
    T           toggle arc turn mode
    P           toggle closed-loop PID (needs ENABLE_ENCODERS=1)
    F1          EMERGENCY STOP (latching)
    F2          reset / clear E-stop

Speed (one-shot):
    =  or  +    speed up
    -           speed down
    R           rough step mode (10%)
    N           normal step mode (5%)
    F           fine step mode (1%)

Log channels (hold SHIFT, tap a letter):
    SHIFT + C   toggle CH_COM  (Comms)
    SHIFT + I   toggle CH_INP  (Input)
    SHIFT + M   toggle CH_MOT  (Motor)
    SHIFT + R   toggle CH_RMP  (Ramp)
    SHIFT + P   toggle CH_PID  (PID)
    SHIFT + S   toggle CH_SNR  (Sensors)
    SHIFT + F   toggle CH_SAF  (Safety)
    SHIFT + W   toggle CH_WDG  (Watchdog)
    SHIFT + G   verbose dump (all masks)
    SHIFT + =   all channels ON
    SHIFT + -   all channels OFF

Log levels (hold SHIFT, tap a digit):
    SHIFT + 1   toggle D level (Debug)
    SHIFT + 2   toggle I level (Info)
    SHIFT + 3   toggle W level (Warn)
    SHIFT + 4   toggle E level (Error)

    Shift suppresses motion while held. Release Shift to drive again.

Quit:
    ESC         sends stop, closes port
"""

import argparse
import ctypes
import sys
import time
import threading

import serial

WINDOWS = sys.platform == 'win32'

if WINDOWS:
    # Virtual key codes for GetAsyncKeyState
    VK = {
        # Movement keys
        'W': 0x57, 'A': 0x41, 'S': 0x53, 'D': 0x44,
        'Q': 0x51, 'E': 0x45, 'Z': 0x5A, 'C': 0x43,
        'J': 0x4A, 'L': 0x4C,
        'SPACE': 0x20,

        # System commands
        'ZERO': 0x30, 'ONE': 0x31,
        'TWO':   0x32, 'THREE': 0x33, 'FOUR': 0x34,
        'T': 0x54, 'P': 0x50,
        'F1': 0x70, 'F2': 0x71,
        'TICK': 0xC0,     # backquote / tilde key — watchdog test

        # Speed commands
        'PLUS':  0xBB,   # OEM '+' key
        'MINUS': 0xBD,   # OEM '-' key
        'R': 0x52, 'N': 0x4E, 'F': 0x46,

        # Log modifier and log letters (Shift + letter)
        'SHIFT': 0x10,
        'G':     0x47,
        'I':     0x49,
        'M':     0x4D,

        # Quit
        'ESC': 0x1B,
    }
else:
    VK = {}

# Shift-modified log commands. Firmware reads 'G' then a channel letter
# from the same buffer pass. Send both chars as one frame.
LOG_KEYS = {
    'C': 'GC',   # CH_COM
    'I': 'GI',   # CH_INP
    'M': 'GM',   # CH_MOT
    'R': 'GR',   # CH_RMP
    'P': 'GP',   # CH_PID
    'S': 'GS',   # CH_SNR
    'F': 'GF',   # CH_SAF
    'W': 'GW',   # CH_WDG
    'G': 'GG',   # verbose dump

    # Shift + 1/2/3/4 -> level toggles
    'ONE':   'GLD',
    'TWO':   'GLI',
    'THREE': 'GLW',
    'FOUR':  'GLE',
}

LOG_SPECIAL = {
    'PLUS':  'G+',   # all channels ON
    'MINUS': 'G-',   # all channels OFF
}


# Motion keys, in send order. Every held key is sent each frame.
MOTION_KEYS = [
    ('W', 'W'), ('A', 'A'), ('S', 'S'), ('D', 'D'),
    ('Q', 'Q'), ('E', 'E'), ('Z', 'Z'), ('C', 'C'),
    ('J', 'J'), ('L', 'L'),
]

# One-shot keys. Value is the exact string to send.
# Multi-char strings are fine — the firmware drains the whole
# serial buffer per parse pass, so '%+' sends in one pass.
ONESHOTS = {
    # System
    'ZERO':  '0',
    'ONE':   '1',
    'T':     'T',
    'P':     'P',
    'F1':    '!',     # EMERGENCY STOP (latching)
    'F2':    '?',     # Reset / clear E-stop

    # Speed
    'PLUS':  '%+',
    'MINUS': '%-',
    'R':     '%R',
    'N':     '%N',
    'F':     '%F',
}

# Keys that must not be treated as one-shot until released.
# Prevents the F1 E-stop from firing on every poll while held.
EDGE_ONLY = set(ONESHOTS.keys())

HEARTBEAT_MS = 40   # 25 Hz. Watchdog timeout is 150 ms.
POLL_MS = 20        # keyboard poll

class Driver:
    def __init__(self, port, baud, quiet):
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.1)
        self.quiet = quiet
        self.lock = threading.Lock()
        self.active_keys = 'X'   # one or more chars, sent as a single frame
        self.pending_oneshots = []
        self.stop = False
        self._drop_until = 0.0

    # ---------- Windows key state ----------
    def _pressed(self, name):
        if not WINDOWS:
            return False
        return ctypes.windll.user32.GetAsyncKeyState(VK[name]) & 0x8000 != 0

    # ---------- Threads ----------
    def _keyboard_poll(self):
        prev = {k: False for k in VK}
        prev.update({('LOG_' + k): False for k in LOG_KEYS})
        prev['LOG_PLUS']  = False
        prev['LOG_MINUS'] = False

        while not self.stop:
            shift = self._pressed('SHIFT')

            # Motion suppressed while Shift is held.
            if shift:
                keys = 'X'
            elif self._pressed('SPACE'):
                keys = 'X'
            else:
                held = [char for name, char in MOTION_KEYS if self._pressed(name)]
                keys = ''.join(held) if held else 'X'

            with self.lock:
                self.active_keys = keys

            # Shift + letter -> 'G' + letter
            for letter, cmd in LOG_KEYS.items():
                now = self._pressed(letter)
                pkey = 'LOG_' + letter
                if shift and now and not prev[pkey]:
                    with self.lock:
                        self.pending_oneshots.append(('cmd', cmd))
                prev[pkey] = now

            # Shift + '+' / '-' -> all channels on / off
            for name, cmd in LOG_SPECIAL.items():
                now = self._pressed(name)
                pkey = 'LOG_' + name
                if shift and now and not prev[pkey]:
                    with self.lock:
                        self.pending_oneshots.append(('cmd', cmd))
                prev[pkey] = now

            # Plain one-shots (system, speed). Skipped while Shift is held.
            if not shift:
                for name in EDGE_ONLY:
                    now = self._pressed(name)
                    if now and not prev.get(name, False):
                        with self.lock:
                            self.pending_oneshots.append(('cmd', ONESHOTS[name]))
                    prev[name] = now

            # ` (backquote) = drop heartbeat for 500 ms (watchdog test)
            now = self._pressed('TICK')
            if now and not prev.get('TICK', False):
                self._drop_until = time.monotonic() + 0.5
                sys.stdout.write("\r[TEST] heartbeat dropped 500 ms\n")
                sys.stdout.flush()
            prev['TICK'] = now

            # ESC quit
            now = self._pressed('ESC')
            if now and not prev.get('ESC', False):
                with self.lock:
                    self.pending_oneshots.append(('quit', None))
            prev['ESC'] = now

            time.sleep(POLL_MS / 1000.0)

    def _heartbeat(self):
        while not self.stop:
            if time.monotonic() < self._drop_until:
                time.sleep(0.01)
                continue
            with self.lock:
                keys = self.active_keys
                oneshots = self.pending_oneshots
                self.pending_oneshots = []

            try:
                # One write for all held keys. The firmware parser drains
                # the whole buffer per pass, so a multi-char frame becomes
                # one combined MotionCommand.
                self.ser.write(keys.encode('ascii'))
                for kind, cmd in oneshots:
                    if kind == 'quit':
                        self.stop = True
                        break
                    self.ser.write(cmd.encode('ascii'))
            except serial.SerialException as e:
                print(f"\nSerial error: {e}", file=sys.stderr)
                self.stop = True
                return

            if self.stop:
                return

            time.sleep(HEARTBEAT_MS / 1000.0)

    def _reader(self):
        buf = b''
        last = ""
        while not self.stop:
            try:
                chunk = self.ser.read(64)
            except serial.SerialException:
                return
            if not chunk:
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                text = line.decode('ascii', errors='ignore').strip()
                if not text or self.quiet:
                    continue
                if text == last:
                    continue
                last = text
                sys.stdout.write(f"\r{text[:78]:<78}\n")
                sys.stdout.flush()

    # ---------- Lifecycle ----------
    def run(self):
        threads = [
            threading.Thread(target=self._heartbeat, daemon=True),
            threading.Thread(target=self._reader, daemon=True),
            threading.Thread(target=self._keyboard_poll, daemon=True),
        ]
        for t in threads:
            t.start()

        print(f"Connected: {self.ser.name} @ {self.ser.baudrate}")
        print("Move:  W A S D   Q E Z C   J L    Stop: SPACE   Quit: ESC")
        print("Mode:  0 = manual, 1 = autonomous, T = arc turn, P = PID")
        print("Speed: = / -   step mode: R / N / F")
        print("E-stop: F1      Reset: F2      Watchdog test: `")
        print("Log:   SHIFT + C/I/M/R/P/S/F/W/G = channels")
        print("       SHIFT + 1/2/3/4 = D/I/W/E levels")
        print("       SHIFT+= all on   SHIFT+- all off")
        print()

        try:
            while not self.stop:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _shutdown(self):
        self.stop = True
        time.sleep(0.2)
        try:
            for _ in range(3):
                self.ser.write(b'X')
                time.sleep(0.03)
            self.ser.close()
        except Exception:
            pass
        print("Closed.")

def main():
    ap = argparse.ArgumentParser(description="Keyboard driver for MecanumCar")
    ap.add_argument('port', help='Serial port (e.g. COM5)')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--quiet', action='store_true',
                    help='Suppress incoming telemetry')
    args = ap.parse_args()

    if not WINDOWS:
        print("This script uses Windows GetAsyncKeyState.", file=sys.stderr)
        print("Use the Bluetooth Electronics app on Linux/macOS.", file=sys.stderr)
        sys.exit(1)

    try:
        drv = Driver(args.port, args.baud, args.quiet)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    drv.run()

if __name__ == '__main__':
    main()
