#!/usr/bin/env python3
"""
Keyboard driver for Project MecanumCar over Bluetooth serial.

Sends motion commands to the HC-05/HC-06 at 25 Hz and auto-releases
when the key is no longer held. Uses GetAsyncKeyState so press-and-hold
works without OS key-repeat delay.

Fixes:
  1. Drains the console input buffer so keys do not leak to the shell prompt.
  2. Locks the controls at the top of the terminal using ANSI scroll regions,
     so telemetry updates below without scrolling the controls out of view.

Usage:
    python drive.py COM5
    python drive.py COM5 --baud 9600
    python drive.py COM5 --quiet

Keys (hold for continuous motion, combine freely):
    W A S D     forward, strafe left, backward, strafe right
    Q E Z C     diagonals (FL, FR, BL, BR)
    J L         spin CCW / CW
    SPACE       stop (overrides any motion key)

System (one-shot):
    1           manual mode
    2           autonomous mode
    T           toggle arc turn mode
    P           toggle closed-loop PID
    F1          EMERGENCY STOP (latching)
    F2          reset / clear user faults

Speed (one-shot):
    =  or  +    speed up
    -           speed down
    R           rough step mode (10%)
    N           normal step mode (5%)
    F           fine step mode (1%)

Log channels (hold SHIFT, tap a letter):
    SHIFT + C/I/M/R/P/S/F/W     toggle CH_*
    SHIFT + G                   verbose dump
    SHIFT + =                   all channels ON
    SHIFT + -                   all channels OFF

Log levels (hold SHIFT, tap a digit):
    SHIFT + 1/2/3/4             toggle D/I/W/E level

Watchdog test:
    ` (backquote)               drop heartbeat for 500 ms

Quit:
    ESC                         sends stop, closes port
"""

import argparse
import ctypes
import shutil
import sys
import time
import threading

import serial

WINDOWS = sys.platform == 'win32'

if WINDOWS:
    import msvcrt
    # Enable ANSI escape sequences in Windows terminal
    kernel32 = ctypes.windll.kernel32
    kernel32.SetConsoleMode(kernel32.GetStdHandle(-11), 7)
    
    VK = {
        'W': 0x57, 'A': 0x41, 'S': 0x53, 'D': 0x44,
        'Q': 0x51, 'E': 0x45, 'Z': 0x5A, 'C': 0x43,
        'J': 0x4A, 'L': 0x4C,
        'SPACE': 0x20, 'ONE': 0x31, 'TWO':   0x32,
        'THREE': 0x33, 'FOUR': 0x34,
        'T': 0x54, 'P': 0x50,
        'F1': 0x70, 'F2': 0x71,
        'TICK': 0xC0,
        'PLUS':  0xBB,
        'MINUS': 0xBD,
        'R': 0x52, 'N': 0x4E, 'F': 0x46,
        'SHIFT': 0x10,
        'G':     0x47,
        'I':     0x49,
        'M':     0x4D,
        'ESC': 0x1B,
    }
else:
    msvcrt = None
    VK = {}

LOG_KEYS = {
    'C': 'GC', 'I': 'GI', 'M': 'GM', 'R': 'GR',
    'P': 'GP', 'S': 'GS', 'F': 'GF', 'W': 'GW',
    'G': 'GG', 'ONE': 'GLD', 'TWO': 'GLI', 
    'THREE': 'GLW', 'FOUR': 'GLE',
}

LOG_SPECIAL = {
    'PLUS':  'G+',
    'MINUS': 'G-',
}

MOTION_KEYS = [
    ('W', 'W'), ('A', 'A'), ('S', 'S'), ('D', 'D'),
    ('Q', 'Q'), ('E', 'E'), ('Z', 'Z'), ('C', 'C'),
    ('J', 'J'), ('L', 'L'),
]

ONESHOTS = {
    'ONE':   '0',  # Key 1 sends Manual (0)
    'TWO':   '1',  # Key 2 sends Autonomous (1)
    'T':     'T',
    'P':     'P',
    'F1':    '!',
    'F2':    '?',
    'PLUS':  '%+',
    'MINUS': '%-',
    'R':     '%R',
    'N':     '%N',
    'F':     '%F',
}

EDGE_ONLY = set(ONESHOTS.keys())

HEARTBEAT_MS = 40
POLL_MS = 20

CONTROLS_TEXT = """\
================================================================================
  MOTION:  W A S D   Q E Z C   J L    |    SPACE stop    |    ESC quit
  MODE:    1 Manual   2 Autonomous   T Arc turn   P PID
  SPEED:   = / -     STEP: R / N / F
  SAFETY:  F1 E-stop    F2 Reset    ` Watchdog drop
  LOG:     Shift + C I M R P S F W G   channels   Shift + 1 2 3 4   levels
================================================================================
"""

class Driver:
    def __init__(self, port, baud, quiet):
        self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.1)
        self.quiet = quiet
        self.lock = threading.Lock()
        self.active_keys = 'X'
        self.pending_oneshots = []
        self.stop = False
        self._drop_until = 0.0
        self._sent_stop = False

        # Terminal layout
        self.term_size = shutil.get_terminal_size()
        self.scroll_start = 9  # Line where telemetry begins
        self.scroll_end = self.term_size.lines - 1

    def _pressed(self, name):
        if not WINDOWS:
            return False
        return ctypes.windll.user32.GetAsyncKeyState(VK[name]) & 0x8000 != 0

    def _drain_stdin(self):
        """Consume console input buffer so keystrokes do not appear on the prompt."""
        while not self.stop:
            if msvcrt:
                while msvcrt.kbhit():
                    msvcrt.getch()
            time.sleep(0.01)

    def _setup_terminal(self):
        if WINDOWS:
            # Clear screen, move to home
            sys.stdout.write("\033[2J\033[H")
            # Print controls
            print(CONTROLS_TEXT)
            # Set scroll region for telemetry
            sys.stdout.write(f"\033[{self.scroll_start};{self.scroll_end}r")
            sys.stdout.write(f"\033[{self.scroll_start};1H")
            sys.stdout.flush()

    def _restore_terminal(self):
        if WINDOWS:
            # Reset scroll region and move cursor to bottom
            sys.stdout.write("\033[r")
            sys.stdout.write(f"\033[{self.term_size.lines};1H\n")
            sys.stdout.flush()

    def _keyboard_poll(self):
        prev = {k: False for k in VK}
        prev.update({('LOG_' + k): False for k in LOG_KEYS})
        prev['LOG_PLUS']  = False
        prev['LOG_MINUS'] = False

        while not self.stop:
            shift = self._pressed('SHIFT')

            if shift:
                keys = 'X'
            elif self._pressed('SPACE'):
                keys = 'X'
            else:
                held = [char for name, char in MOTION_KEYS if self._pressed(name)]
                keys = ''.join(held) if held else 'X'

            with self.lock:
                self.active_keys = keys

            for letter, cmd in LOG_KEYS.items():
                now = self._pressed(letter)
                pkey = 'LOG_' + letter
                if shift and now and not prev[pkey]:
                    with self.lock:
                        self.pending_oneshots.append(('cmd', cmd))
                prev[pkey] = now

            for name, cmd in LOG_SPECIAL.items():
                now = self._pressed(name)
                pkey = 'LOG_' + name
                if shift and now and not prev[pkey]:
                    with self.lock:
                        self.pending_oneshots.append(('cmd', cmd))
                prev[pkey] = now

            if not shift:
                for name in EDGE_ONLY:
                    now = self._pressed(name)
                    if now and not prev.get(name, False):
                        with self.lock:
                            self.pending_oneshots.append(('cmd', ONESHOTS[name]))
                    prev[name] = now

            now = self._pressed('TICK')
            if now and not prev.get('TICK', False):
                self._drop_until = time.monotonic() + 0.5
                with self.lock:
                    self.pending_oneshots.append(('log', "[TEST] heartbeat dropped 500 ms"))
            prev['TICK'] = now

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

            payload = b""
            
            if keys != 'X':
                payload += keys.encode('ascii')
                self._sent_stop = False
            elif not self._sent_stop:
                payload += b'X'
                self._sent_stop = True

            for kind, cmd in oneshots:
                if kind == 'quit':
                    self.stop = True
                    break
                elif kind == 'log':
                    print(cmd, flush=True)
                else:
                    payload += cmd.encode('ascii')

            if payload:
                try:
                    self.ser.write(payload)
                except serial.SerialException as e:
                    print(f"Serial error: {e}", file=sys.stderr, flush=True)
                    self.stop = True
                    return

            if self.stop:
                return

            time.sleep(HEARTBEAT_MS / 1000.0)

    def _reader(self):
        buf = b''
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
                
                # Print inside the scroll region
                print(text, flush=True)

    def run(self):
        self._setup_terminal()
        
        threads = [
            threading.Thread(target=self._drain_stdin, daemon=True),
            threading.Thread(target=self._heartbeat, daemon=True),
            threading.Thread(target=self._reader, daemon=True),
            threading.Thread(target=self._keyboard_poll, daemon=True),
        ]
        for t in threads:
            t.start()

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
        self._restore_terminal()
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
        sys.exit(1)

    try:
        drv = Driver(args.port, args.baud, args.quiet)
    except serial.SerialException as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    drv.run()

if __name__ == '__main__':
    main()
