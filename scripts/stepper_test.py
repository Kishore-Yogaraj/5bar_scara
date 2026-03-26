"""
stepper_test.py  —  Stepper / encoder interaction test for the 5-bar SCARA.

Sequence (repeated LOOP_COUNT times):
  1. Move arms to ARMS_IN position  (tuck)
  2. Move stepper CW by STEPPER_DEG degrees
  3. Move arms to ARMS_OUT position (reach)
  4. Move stepper CCW by STEPPER_DEG degrees  (return to start)

After each arm move the final encoder reading (target vs actual) is printed
so you can see whether the encoders are drifting during stepper operation.

Usage
-----
    python stepper_test.py                 # default COM3
    python stepper_test.py --port COM4
    python stepper_test.py --sim           # no serial, dry-run only
"""

from __future__ import annotations

import sys
import re
import time
import logging
import argparse
from pathlib import Path

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))

from kinematics.ik_solver    import angles_to_commands
from kinematics.serial_comms import SerialComms, DEFAULT_PORT, DEFAULT_BAUDRATE

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG — edit here
# ─────────────────────────────────────────────────────────────────────────────

CONFIG = {
    "port":     DEFAULT_PORT,
    "baudrate": DEFAULT_BAUDRATE,

    # Arms-in (tuck) angles — IK theta degrees from +x axis
    "arms_in_theta1_deg":  125.0,
    "arms_in_theta2_deg":   55.0,

    # Arms-out (reach) angles — IK theta degrees from +x axis
    "arms_out_theta1_deg":  95.0,
    "arms_out_theta2_deg":  85.0,

    # Stepper travel per cycle (degrees).  Positive = CW, negative = CCW.
    # The script goes CW on step 2 and CCW on step 4 to return home.
    "stepper_deg": 45.0,

    # How many full cycles to run
    "loop_count": 5,

    # Timeouts (seconds)
    "arm_move_timeout":    15.0,
    "stepper_timeout":     30.0,
}

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("stepper_test")

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

_ENC_RE = re.compile(
    r"t1=([\d.\-]+)\s+p1=([\d\-]+)\s+t2=([\d.\-]+)\s+p2=([\d\-]+)"
)

def parse_enc(line: str) -> dict | None:
    """Parse an 'ENC t1=X p1=Y t2=X p2=Y' line into a dict, or None."""
    m = _ENC_RE.search(line)
    if not m:
        return None
    return {
        "t1": float(m.group(1)), "p1": int(m.group(2)),
        "t2": float(m.group(3)), "p2": int(m.group(4)),
    }


def wait_done(comms: SerialComms | None, timeout: float, label: str) -> dict | None:
    """
    Block until ESP32 sends 'DONE' (or timeout).

    Returns the *last* ENC reading seen before DONE, or None if none arrived.
    In simulation mode (comms is None) returns a fake zero reading immediately.
    """
    if comms is None:
        log.info("  [SIM] %s — skipping wait.", label)
        return {"t1": 0.0, "p1": 0, "t2": 0.0, "p2": 0}

    deadline = time.time() + timeout
    last_enc: dict | None = None

    while time.time() < deadline:
        if comms._ser and comms._ser.in_waiting:
            try:
                raw  = comms._ser.readline()
                line = raw.decode("utf-8", errors="replace").rstrip()
            except Exception as exc:
                log.error("  Read error: %s", exc)
                break

            if not line:
                continue

            enc = parse_enc(line)
            if enc:
                last_enc = enc
                log.info("  [ENC] t1=%+.0f  p1=%+d  err1=%+.0f  |  "
                         "t2=%+.0f  p2=%+d  err2=%+.0f",
                         enc["t1"], enc["p1"], enc["p1"] - enc["t1"],
                         enc["t2"], enc["p2"], enc["p2"] - enc["t2"])
            elif "DONE" in line:
                log.info("  [ESP32] %s", line)
                return last_enc
            else:
                log.info("  [ESP32] %s", line)

    log.warning("  Timed out waiting for DONE (%s, %.1fs).", label, timeout)
    return last_enc


def send_arm_move(
    comms: SerialComms | None,
    theta1_deg: float,
    theta2_deg: float,
    label: str,
    timeout: float,
) -> dict | None:
    """Send arm-angle command and wait for DONE.  Returns last ENC dict."""
    cmd1, cmd2 = angles_to_commands(theta1_deg, theta2_deg)
    log.info("  %-14s  θ1=%+.1f°  θ2=%+.1f°  cmd=(%+.2f, %+.2f)",
             label, theta1_deg, theta2_deg, cmd1, cmd2)

    if comms is not None:
        ok = comms.send_command(cmd1, cmd2, 0)
        if not ok:
            log.error("  Serial write failed.")
            return None

    return wait_done(comms, timeout, label)


def send_step(
    comms: SerialComms | None,
    degrees: float,
    label: str,
    timeout: float,
) -> None:
    """Send STEP:<degrees> command and wait for DONE."""
    direction = "CW" if degrees >= 0 else "CCW"
    log.info("  %-14s  %.1f°  (%s)", label, abs(degrees), direction)

    if comms is None:
        log.info("  [SIM] STEP not sent.")
        return

    try:
        msg = f"STEP:{degrees:.2f}\n"
        comms._ser.write(msg.encode())
        comms._ser.flush()
    except Exception as exc:
        log.error("  STEP write failed: %s", exc)
        return

    wait_done(comms, timeout, label)


def report_result(label: str, enc: dict | None) -> None:
    """Print a summary line showing target vs actual encoder ticks."""
    if enc is None:
        log.info("  %-14s  no encoder data received.", label)
        return

    err1 = enc["p1"] - enc["t1"]
    err2 = enc["p2"] - enc["t2"]
    log.info(
        "  %-14s  M1: target=%+.0f  actual=%+d  error=%+.0f  |  "
        "M2: target=%+.0f  actual=%+d  error=%+.0f",
        label + " RESULT",
        enc["t1"], enc["p1"], err1,
        enc["t2"], enc["p2"], err2,
    )

# ─────────────────────────────────────────────────────────────────────────────
# Main test loop
# ─────────────────────────────────────────────────────────────────────────────

def run(cfg: dict, comms: SerialComms | None) -> None:
    loops       = cfg["loop_count"]
    arm_timeout = cfg["arm_move_timeout"]
    stp_timeout = cfg["stepper_timeout"]
    stp_deg     = cfg["stepper_deg"]

    log.info("=" * 60)
    log.info("Stepper / encoder test — %d cycle(s)", loops)
    log.info("  Arms-in :  θ1=%.1f°  θ2=%.1f°",
             cfg["arms_in_theta1_deg"], cfg["arms_in_theta2_deg"])
    log.info("  Arms-out:  θ1=%.1f°  θ2=%.1f°",
             cfg["arms_out_theta1_deg"], cfg["arms_out_theta2_deg"])
    log.info("  Stepper :  ±%.1f°", stp_deg)
    log.info("=" * 60)

    for cycle in range(1, loops + 1):
        log.info("")
        log.info("── Cycle %d / %d ─────────────────────────────────────────", cycle, loops)

        # 1. Arms in (tuck)
        log.info("Step 1: Arms IN")
        enc_in = send_arm_move(
            comms,
            cfg["arms_in_theta1_deg"],
            cfg["arms_in_theta2_deg"],
            "ARMS_IN",
            arm_timeout,
        )
        report_result("ARMS_IN", enc_in)

        # 2. Stepper CW
        log.info("Step 2: Stepper CW %.1f°", stp_deg)
        send_step(comms, stp_deg, "STEP_CW", stp_timeout)

        # 3. Arms out (reach)
        log.info("Step 3: Arms OUT")
        enc_out = send_arm_move(
            comms,
            cfg["arms_out_theta1_deg"],
            cfg["arms_out_theta2_deg"],
            "ARMS_OUT",
            arm_timeout,
        )
        report_result("ARMS_OUT", enc_out)

        # 4. Stepper CCW (return)
        log.info("Step 4: Stepper CCW %.1f° (return)", stp_deg)
        send_step(comms, -stp_deg, "STEP_CCW", stp_timeout)

    log.info("")
    log.info("=" * 60)
    log.info("Test complete.")
    log.info("=" * 60)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Stepper / encoder interaction test")
    p.add_argument("--port",   type=str, default=None,
                   help="Serial port (e.g. COM4 or /dev/ttyUSB0)")
    p.add_argument("--sim",    action="store_true",
                   help="Simulation mode — log commands but send nothing")
    p.add_argument("--loops",  type=int, default=None,
                   help="Override number of cycles")
    args = p.parse_args()

    if args.port:
        CONFIG["port"] = args.port
    if args.loops is not None:
        CONFIG["loop_count"] = args.loops

    comms: SerialComms | None = None

    if args.sim:
        log.info("Simulation mode — no serial commands will be sent.")
    else:
        log.info("Connecting to ESP32 on %s …", CONFIG["port"])
        comms = SerialComms(CONFIG["port"], CONFIG["baudrate"])
        if not comms.connect():
            log.warning("Could not open %s — falling back to simulation mode.", CONFIG["port"])
            comms = None
        else:
            log.info("Connected.")

    try:
        run(CONFIG, comms)
    except KeyboardInterrupt:
        log.info("Interrupted.")
    finally:
        if comms and comms.is_connected:
            comms.disconnect()
