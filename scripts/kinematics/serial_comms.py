"""
serial_comms.py  —  Serial communication layer for the 5-bar SCARA robot.

Wraps the ESP32 serial protocol behind a clean API so that orchestrators
(pick_and_place.py) and visualisation scripts (send_ik_animation.py) never
touch pyserial directly.

Public API
----------
SerialComms(port, baudrate)     — connection manager (context manager support)
    .connect()                  — open port, returns True on success
    .disconnect()               — flush & close
    .send_command(cmd1, cmd2)   — send "cmd1,cmd2\n" to firmware
    .send_home()                — send "HOME\n" to reset encoder zero
    .drain(timeout)             — read & yield lines until timeout expires
    .wait_for_done(keyword, timeout) — block until firmware echoes keyword
    .is_connected               — bool property

Module-level helpers
--------------------
connect(port, baudrate)         — shorthand; returns a connected SerialComms
                                  or None if the port is unavailable.
"""

import serial
import time
import logging
from typing import Generator

log = logging.getLogger(__name__)

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_PORT     = "COM3"
DEFAULT_BAUDRATE = 115200
BOOT_DELAY       = 2.0    # seconds to wait after opening port (ESP32 reset)
READ_TIMEOUT     = 0.05   # pyserial read timeout (seconds)


# ─────────────────────────────────────────────────────────────────────────────
# SerialComms class
# ─────────────────────────────────────────────────────────────────────────────

class SerialComms:
    """
    Manages a persistent serial connection to the ESP32 firmware.

    Usage — explicit:
        comms = SerialComms("COM3", 115200)
        comms.connect()
        comms.send_command(-12.5, 8.3)
        comms.disconnect()

    Usage — context manager (recommended):
        with SerialComms("COM3") as comms:
            comms.send_command(-12.5, 8.3)
            for line in comms.drain(timeout=3.0):
                print(line)
    """

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUDRATE):
        self.port     = port
        self.baudrate = baudrate
        self._ser: serial.Serial | None = None

    # ── Connection lifecycle ──────────────────────────────────────────────────

    def connect(self, boot_delay: float = BOOT_DELAY) -> bool:
        """
        Open the serial port and wait for the ESP32 to finish booting.

        Returns True on success, False if the port could not be opened
        (logs the exception at WARNING level so callers can decide whether
        to treat it as fatal).
        """
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=READ_TIMEOUT)
            time.sleep(boot_delay)
            log.info("Connected to ESP32 on %s @ %d baud.", self.port, self.baudrate)
            return True
        except serial.SerialException as exc:
            self._ser = None
            log.warning("Could not open %s: %s", self.port, exc)
            return False

    def disconnect(self):
        """Flush any pending bytes and close the port gracefully."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.flush()
                self._ser.close()
                log.info("Serial port %s closed.", self.port)
            except serial.SerialException as exc:
                log.warning("Error closing %s: %s", self.port, exc)
        self._ser = None

    @property
    def is_connected(self) -> bool:
        """True if the port is open and ready."""
        return self._ser is not None and self._ser.is_open

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self) -> "SerialComms":
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    # ── Commands ──────────────────────────────────────────────────────────────

    def send_command(self, cmd1: float, cmd2: float) -> bool:
        """
        Send motor angle deltas to the firmware.

        The wire format is:  "<cmd1>,<cmd2>\\n"
        where cmd1 / cmd2 are signed floating-point degrees relative to the
        physical home position, produced by ik_solver.angles_to_commands().

        Returns True if the write succeeded, False otherwise.
        """
        if not self.is_connected:
            log.error("send_command called but serial port is not open.")
            return False

        message = f"{cmd1:.2f},{cmd2:.2f}\n"
        try:
            self._ser.write(message.encode())
            self._ser.flush()
            log.debug("Sent: %s", message.strip())
            return True
        except serial.SerialException as exc:
            log.error("Write failed: %s", exc)
            return False

    def send_home(self) -> bool:
        """
        Send the HOME command to reset encoder counts to zero.

        The physical arms *must* be at the home position before calling this,
        otherwise all subsequent IK commands will be offset incorrectly.

        Returns True if the write succeeded.
        """
        if not self.is_connected:
            log.error("send_home called but serial port is not open.")
            return False

        try:
            self._ser.write(b"HOME\n")
            self._ser.flush()
            log.info("HOME command sent — encoders reset to zero.")
            return True
        except serial.SerialException as exc:
            log.error("HOME write failed: %s", exc)
            return False

    # ── Reading ───────────────────────────────────────────────────────────────

    def drain(self, timeout: float = 3.0) -> Generator[str, None, None]:
        """
        Yield decoded lines from the ESP32 until *timeout* seconds have
        elapsed since the call (not since the last byte received).

        Example:
            comms.send_command(cmd1, cmd2)
            for line in comms.drain(timeout=3.0):
                print("[ESP32]", line)
        """
        if not self.is_connected:
            return

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ser.in_waiting:
                try:
                    raw  = self._ser.readline()
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    if line:
                        log.debug("[ESP32] %s", line)
                        yield line
                except serial.SerialException as exc:
                    log.error("Read error: %s", exc)
                    return

    def wait_for_done(self, keyword: str = "DONE", timeout: float = 10.0) -> bool:
        """
        Block until the firmware echoes a line containing *keyword* or until
        *timeout* seconds elapse.

        Useful for move-then-wait orchestration in pick_and_place.py:

            comms.send_command(cmd1, cmd2)
            if not comms.wait_for_done("DONE", timeout=8.0):
                raise RuntimeError("Robot did not complete move in time.")

        Returns True if the keyword was found, False on timeout.

        Note: this requires the ESP32 firmware to send a line containing
        the keyword when the move is complete.  If the firmware does not
        yet do this, use drain() with a fixed timeout instead.
        """
        if not self.is_connected:
            return False

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._ser.in_waiting:
                try:
                    raw  = self._ser.readline()
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    if line:
                        log.debug("[ESP32] %s", line)
                        if keyword in line:
                            return True
                except serial.SerialException as exc:
                    log.error("Read error during wait_for_done: %s", exc)
                    return False

        log.warning("wait_for_done timed out after %.1f s (keyword=%r).", timeout, keyword)
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Module-level convenience function
# ─────────────────────────────────────────────────────────────────────────────

def connect(
    port: str     = DEFAULT_PORT,
    baudrate: int = DEFAULT_BAUDRATE,
) -> "SerialComms | None":
    """
    Attempt to open a serial connection and return a ready SerialComms instance.
    Returns None if the port is unavailable, so callers can fall back to
    simulation mode gracefully.

    Example:
        comms = serial_comms.connect()
        if comms is None:
            print("Running in simulation mode — no serial output.")
    """
    comms = SerialComms(port, baudrate)
    if comms.connect():
        return comms
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Quick self-test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s  %(message)s")

    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    print(f"\nAttempting connection on {port} …\n")

    comms = connect(port)
    if comms is None:
        print("No serial connection — self-test skipped.")
        print("Pass the correct COM port as an argument:  python serial_comms.py COM4")
        sys.exit(0)

    with comms:
        print("Connection OK.  Sending test command: 0.00, 0.00  (home position)\n")
        comms.send_command(0.0, 0.0)

        print("Draining ESP32 output for 3 s …")
        for line in comms.drain(timeout=3.0):
            print(f"  [ESP32] {line}")

        print("\nDone.")