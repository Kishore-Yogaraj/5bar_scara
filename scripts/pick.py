"""
pick.py  —  Position-visit orchestrator for the 5-bar parallel SCARA robot.

Workflow
--------
1. Wait STARTUP_DELAY seconds (time to get clear of the robot)
2. Capture a frame and run object detection
3. For each detected object in order:
       a. Solve IK for the object position
       b. Move the robot to that position
       c. Dwell for DWELL_SECONDS at the position
4. After all objects have been visited once, stop

Press Q or ESC in the preview window to quit early.

Usage
-----
    python pick.py                   # default config
    python pick.py --port COM4       # override serial port
    python pick.py --no-preview      # headless / no cv2 window
    python pick.py --sim             # force simulation mode (no serial)
"""

from __future__ import annotations

import sys
import time
import logging
import argparse
import cv2
import numpy as np
from pathlib import Path
from enum import Enum, auto

# ── Path setup: allow running from scripts/ or project root ──────────────────
_HERE = Path(__file__).parent           # scripts/
sys.path.insert(0, str(_HERE))
from kinematics.ik_solver    import solve_ik, angles_to_commands
from kinematics.serial_comms import SerialComms, DEFAULT_PORT, DEFAULT_BAUDRATE
from vision.detector         import (
    load_homography, load_hsv_config, load_intrinsics,
    get_detections, annotate_frame, Detection,
)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG  —  edit here, nowhere else
# ─────────────────────────────────────────────────────────────────────────────

CONFIG = {
    # ── Serial ────────────────────────────────────────────────────────────────
    "port":     DEFAULT_PORT,
    "baudrate": DEFAULT_BAUDRATE,

    # ── Vision asset paths (relative to this file) ───────────────────────────
    "homography": _HERE / "vision" / "homography.json",
    "hsv_config": _HERE / "vision" / "hsv_config.json",
    "intrinsics": _HERE / "vision" / "intrinsics",

    # ── Camera ────────────────────────────────────────────────────────────────
    "camera_index": 0,

    # ── Timing (seconds) ──────────────────────────────────────────────────────
    "startup_delay":      3.0,   # wait at launch before doing anything
    "dwell_seconds":      3.0,   # how long to hold position at each object
    "move_drain_timeout": 15.0,  # how long to wait for ESP32 DONE after each move

    # ── Pick ordering strategy ────────────────────────────────────────────────
    # "nearest"   — nearest to robot origin first (default from get_detections)
    # "red_first" — all red objects before blue
    # "blue_first"— all blue objects before red
    "pick_order": "nearest",

    # ── Preview window ────────────────────────────────────────────────────────
    "show_preview": True,
}

# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("pick")


# ─────────────────────────────────────────────────────────────────────────────
# State machine
# ─────────────────────────────────────────────────────────────────────────────

class State(Enum):
    STARTUP     = auto()
    SCAN        = auto()
    MOVE_TO_OBJ = auto()
    DWELL       = auto()
    DONE        = auto()


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def move_to(
    x_mm: float,
    y_mm: float,
    comms: SerialComms | None,
    label: str = "",
    drain_timeout: float = 4.0,
) -> bool:
    """
    Solve IK for (x_mm, y_mm) and send the motor command.

    Returns True on success, False if IK fails or the serial write fails.
    In simulation mode (comms is None) always returns True after logging.
    """
    ik = solve_ik(x_mm, y_mm)
    if not ik["valid"]:
        log.warning("IK failed for %s (%.1f, %.1f): %s",
                    label, x_mm, y_mm, ik["reason"])
        return False

    cmd1, cmd2 = angles_to_commands(ik["theta1_deg"], ik["theta2_deg"])
    log.info("  → %s  (%.1f, %.1f) mm  θ1=%.2f°  θ2=%.2f°  cmd=(%.2f, %.2f)",
             label, x_mm, y_mm, ik["theta1_deg"], ik["theta2_deg"], cmd1, cmd2)

    if comms is None:
        log.info("  [SIM] command not sent.")
        return True

    ok = comms.send_command(cmd1, cmd2)
    if not ok:
        log.error("  Serial write failed.")
        return False

    if not comms.wait_for_done("DONE", timeout=drain_timeout):
        log.warning("  Move timed out waiting for DONE (%.1fs).", drain_timeout)

    return True


def order_picks(detections: list[Detection], strategy: str) -> list[Detection]:
    """Return detections sorted by strategy."""
    if strategy == "nearest":
        return list(detections)   # already sorted nearest-first by get_detections
    priority = {"red": 0, "blue": 1} if strategy == "red_first" else {"blue": 0, "red": 1}
    return sorted(
        detections,
        key=lambda d: (priority.get(d["color"], 99), d["x_mm"] ** 2 + d["y_mm"] ** 2),
    )


def countdown(seconds: float, message: str, cap: cv2.VideoCapture,
              show_preview: bool, H: np.ndarray, H_inv: np.ndarray,
              vcfg: dict, K, dist,
              last_detections: list[Detection],
              comms: "SerialComms | None") -> bool:
    """
    Block for *seconds* while printing a live countdown.
    Shows a live annotated feed and handles H (home) and Q/ESC (quit).

    Returns False if the user requested quit, True otherwise.
    """
    deadline = time.time() + seconds
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        print(f"\r  {message}: {remaining:.1f}s …   ", end="", flush=True)

        if show_preview:
            ret, frame = cap.read()
            if ret:
                if K is not None:
                    frame = cv2.undistort(frame, K, dist)
                display = annotate_frame(frame, last_detections, H, H_inv)

                # Countdown overlay
                cv2.putText(display, f"{message}: {remaining:.1f}s",
                            (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(display, f"{message}: {remaining:.1f}s",
                            (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 200), 1, cv2.LINE_AA)
                cv2.imshow("[ pick.py ]", display)

            key = cv2.waitKey(100) & 0xFF
            if key in (ord("q"), 27):
                print()
                return False
            if key == ord("h"):
                if comms and comms.is_connected:
                    comms.send_home()
                    log.info("HOME sent — encoders reset to zero.")
                else:
                    log.warning("HOME requested but no serial connection.")
        else:
            time.sleep(0.1)

    print()
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Main orchestrator
# ─────────────────────────────────────────────────────────────────────────────

def run(cfg: dict, comms: SerialComms | None):
    # ── Load vision assets ────────────────────────────────────────────────────
    H       = load_homography(cfg["homography"])
    H_inv   = np.linalg.inv(H)
    vcfg    = load_hsv_config(cfg["hsv_config"])
    K, dist = load_intrinsics(cfg["intrinsics"])

    # ── Open camera ───────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(cfg["camera_index"])
    if not cap.isOpened():
        log.error("Cannot open camera index %d.", cfg["camera_index"])
        return

    if cfg["show_preview"]:
        cv2.namedWindow("[ pick.py ]", cv2.WINDOW_NORMAL)

    state           : State            = State.STARTUP
    visit_list      : list[Detection]  = []
    current         : Detection | None = None
    total_objs      : int              = 0
    scan_frame      : np.ndarray | None = None
    last_detections : list[Detection]  = []   # kept alive for live annotation

    log.info("Orchestrator started.  Press Q / ESC to quit.  H to home.")

    try:
        while True:

            # ── Global quit check + H to home ────────────────────────────────
            if cfg["show_preview"]:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    log.info("Quit requested.")
                    break
                if key == ord("h"):
                    if comms and comms.is_connected:
                        comms.send_home()
                        log.info("HOME sent — encoders reset to zero.")
                    else:
                        log.warning("HOME requested but no serial connection.")

            # ── STARTUP ───────────────────────────────────────────────────────
            if state == State.STARTUP:
                log.info("Starting in %.0f seconds — get clear of the robot.",
                         cfg["startup_delay"])
                ok = countdown(cfg["startup_delay"], "Starting in",
                               cap, cfg["show_preview"], H, H_inv,
                               vcfg, K, dist, last_detections, comms)
                if not ok:
                    break
                state = State.SCAN

            # ── SCAN ──────────────────────────────────────────────────────────
            elif state == State.SCAN:
                log.info("Scanning …")

                ret, frame = cap.read()
                if not ret:
                    log.error("Failed to capture frame from camera.")
                    break

                if K is not None:
                    frame = cv2.undistort(frame, K, dist)

                detections      = get_detections(frame, H, vcfg)
                visit_list      = order_picks(detections, cfg["pick_order"])
                total_objs      = len(visit_list)
                scan_frame      = frame
                last_detections = list(visit_list)   # snapshot for annotation

                if not visit_list:
                    log.info("No objects detected — nothing to do.")
                    state = State.DONE
                    continue

                log.info("Found %d object(s):", total_objs)
                for i, d in enumerate(visit_list):
                    log.info("  %d. %s  (%.1f, %.1f) mm  angle=%.1f°",
                             i + 1, d["color"], d["x_mm"], d["y_mm"], d["angle_deg"])

                # Show annotated scan frame before the run begins
                if cfg["show_preview"]:
                    display = annotate_frame(scan_frame, visit_list, H, H_inv)
                    cv2.putText(display,
                                f"Found {total_objs} object(s) — starting run",
                                (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(display,
                                f"Found {total_objs} object(s) — starting run",
                                (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 200), 1, cv2.LINE_AA)
                    cv2.imshow("[ pick.py ]", display)
                    cv2.waitKey(1)

                current = visit_list.pop(0)
                state   = State.MOVE_TO_OBJ

            # ── MOVE TO OBJECT ────────────────────────────────────────────────
            elif state == State.MOVE_TO_OBJ:
                visited = total_objs - len(visit_list)
                log.info("── Object %d / %d  (%s) ─────────────────────",
                         visited, total_objs, current["color"])

                ok = move_to(
                    current["x_mm"], current["y_mm"],
                    comms,
                    label=f"OBJ-{visited}",
                    drain_timeout=cfg["move_drain_timeout"],
                )

                if not ok:
                    log.warning("Skipping object %d (IK failed).", visited)
                    if visit_list:
                        current = visit_list.pop(0)
                        # stay in MOVE_TO_OBJ
                    else:
                        state = State.DONE
                else:
                    state = State.DWELL

            # ── DWELL ─────────────────────────────────────────────────────────
            elif state == State.DWELL:
                log.info("  At position — dwelling for %.0fs.", cfg["dwell_seconds"])
                ok = countdown(cfg["dwell_seconds"], "Dwelling",
                               cap, cfg["show_preview"], H, H_inv,
                               vcfg, K, dist, last_detections, comms)
                if not ok:
                    break

                if visit_list:
                    current = visit_list.pop(0)
                    state   = State.MOVE_TO_OBJ
                else:
                    state = State.DONE

            # ── DONE ──────────────────────────────────────────────────────────
            elif state == State.DONE:
                log.info("All %d object(s) visited.  Run complete.", total_objs)
                if cfg["show_preview"]:
                    log.info("Preview open — press Q / ESC to close.")
                    while True:
                        ret, frame = cap.read()
                        if ret:
                            if K is not None:
                                frame = cv2.undistort(frame, K, dist)
                            display = annotate_frame(frame, last_detections, H, H_inv)
                            cv2.putText(display, "Run complete Q to quit",
                                        (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (0, 0, 0), 2, cv2.LINE_AA)
                            cv2.putText(display, "Run complete Q to quit",
                                        (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                        (0, 255, 200), 1, cv2.LINE_AA)
                            cv2.imshow("[ pick.py ]", display)
                        key = cv2.waitKey(30) & 0xFF
                        if key in (ord("q"), 27):
                            break
                        if key == ord("h"):
                            if comms and comms.is_connected:
                                comms.send_home()
                                log.info("HOME sent.")
                break

    except KeyboardInterrupt:
        log.info("Interrupted.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        log.info("Done.")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="5-bar SCARA position-visit orchestrator"
    )
    p.add_argument("--port",       type=str,            default=None,
                   help="Serial port override (e.g. COM4 or /dev/ttyUSB0)")
    p.add_argument("--no-preview", action="store_true",
                   help="Disable the live OpenCV preview window")
    p.add_argument("--sim",        action="store_true",
                   help="Simulation mode — solve IK but send no serial commands")
    args = p.parse_args()

    if args.port:
        CONFIG["port"] = args.port
    if args.no_preview:
        CONFIG["show_preview"] = False

    comms: SerialComms | None = None

    if args.sim:
        log.info("Simulation mode — no serial commands will be sent.")
    else:
        log.info("Connecting to ESP32 on %s …", CONFIG["port"])
        comms = SerialComms(CONFIG["port"], CONFIG["baudrate"])
        if not comms.connect():
            log.warning(
                "Could not open %s — falling back to simulation mode.",
                CONFIG["port"],
            )
            comms = None
        else:
            log.info("Serial connection established.")

    try:
        run(CONFIG, comms)
    finally:
        if comms and comms.is_connected:
            comms.disconnect()