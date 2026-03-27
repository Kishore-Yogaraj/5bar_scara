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
    "startup_delay":       5.0,  # wait at launch before doing anything
    "move_drain_timeout":  15.0, # how long to wait for ESP32 DONE after each arm move
    "rotate_timeout":      30.0, # how long to wait for ESP32 DONE after ROTATE
    "rotate_settle":        3.0, # settle countdown after stepper reaches pick zone
    "rotate_home_timeout": 30.0, # how long to wait for ESP32 DONE after ROTATE_HOME
    "rotate_home_settle":   3.0, # settle countdown after stepper returns to drop zone
    "arms_in_timeout":     10.0, # how long to wait for ESP32 DONE after arms-in move
    "pick_timeout":        15.0, # how long to wait for ESP32 DONE after PICK
    "drop_timeout":        10.0, # how long to wait for ESP32 DONE after DROP
    "rescan_dwell":         5.0, # seconds to dwell at drop zone before re-scanning for new objects

    # ── Arms-in (tuck) angles ──────────────────────────────────────────────────
    # IK angles (degrees from +x axis) the arms move to before the stepper rotates.
    # Adjust these until the tuck position clears any obstacles.
    # HOME_ANGLE is 90° (straight up); values below 90° pull left arm inward,
    # values above 90° pull right arm inward.
    "arms_in_theta1_deg": 125.0,   # left arm tuck angle
    "arms_in_theta2_deg":  55.0,   # right arm tuck angle

    # ── Drop-off positions (by colour) ────────────────────────────────────────
    # Arm angles (degrees) at the drop zone for each detected colour.
    # Adjust until the end-effector is over the correct bin for each colour.
    "drop_blue_theta1_deg":  123.34,  # left arm angle when dropping a red object
    "drop_blue_theta2_deg":   80.60,  # right arm angle when dropping a red object
    "drop_red_theta1_deg": 96.11,  # left arm angle when dropping a blue object
    "drop_red_theta2_deg":  52.22,  # right arm angle when dropping a blue object

    # ── Pick ordering strategy ────────────────────────────────────────────────
    # "nearest"   — nearest to robot origin first (default from get_detections)
    # "red_first" — all red objects before blue
    # "blue_first"— all blue objects before red
    "pick_order": "red_first",

    # ── Quadrant position offset (mm) ─────────────────────────────────────────
    # Applied when a detected object is in the positive-X AND positive-Y quadrant
    # (camera bottom-left corner per calibration convention).
    # Tune these values physically to correct systematic misses caused by lens
    # distortion at the image edge.  Start at 0.0 and adjust in small steps.
    "quadrant_pp_offset_x_mm": -5.0,   # extra mm added to X (positive = right)
    "quadrant_pp_offset_y_mm": -10.0,   # extra mm added to Y (positive = forward)

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
    STARTUP          = auto()
    SCAN             = auto()
    ARMS_IN          = auto()   # tuck arms before stepper rotates to pick zone
    ROTATE_BACK      = auto()   # stepper CW to pick zone
    MOVE_TO_OBJ      = auto()   # arm moves to object; ESP32 rotates gripper on arrival
    PICK_OBJ         = auto()   # send PICK; ESP32 picks up + rotates gripper back
    ARMS_IN_AFTER_PICK = auto() # tuck arms before rotating back to drop zone
    ROTATE_TO_DROP   = auto()   # stepper CCW back to drop zone
    MOVE_TO_DROP     = auto()   # arm moves to drop position
    DROP_OBJ         = auto()   # send DROP; ESP32 releases object
    RESCAN           = auto()   # dwell then re-scan pick zone for new objects after queue drains
    DONE             = auto()


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def move_to(
    x_mm: float,
    y_mm: float,
    comms: SerialComms | None,
    label: str = "",
    drain_timeout: float = 4.0,
    object_angle_deg: float = 0.0,
) -> bool:
    """
    Solve IK for (x_mm, y_mm) and send the motor + servo command.

    Returns True on success, False if IK fails or the serial write fails.
    In simulation mode (comms is None) always returns True after logging.
    """
    ik = solve_ik(x_mm, y_mm)
    if not ik["valid"]:
        log.warning("IK failed for %s (%.1f, %.1f): %s",
                    label, x_mm, y_mm, ik["reason"])
        return False

    cmd1, cmd2 = angles_to_commands(ik["theta1_deg"], ik["theta2_deg"])
    diff = abs(object_angle_deg - ik["phi1_deg"])
    if diff > 180:
        diff = 360 - diff
    if diff > 90:
        diff = 180 - diff
    servo_angle = int(round(diff))
    
    log.info("  → %s  (%.1f, %.1f) mm  θ1=%.2f°  θ2=%.2f°  φ1=%.2f°  servo=%d°  cmd=(%.2f, %.2f)",
             label, x_mm, y_mm, ik["theta1_deg"], ik["theta2_deg"],
             ik["phi1_deg"], servo_angle, cmd1, cmd2)

    if comms is None:
        log.info("  [SIM] command not sent.")
        return True

    ok = comms.send_command(cmd1, cmd2, servo_angle)
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
                state   = State.ARMS_IN

            # ── ARMS IN ───────────────────────────────────────────────────────
            elif state == State.ARMS_IN:
                # # Step 1: move to 90° home position first
                # log.info("Moving arms to home position (90°, 90°) …")
                # home_cmd1, home_cmd2 = angles_to_commands(90.0, 90.0)
                # log.info("  arms-home: θ1=90.0°  θ2=90.0°  cmd=(%.2f, %.2f)",
                #          home_cmd1, home_cmd2)
                # if comms is not None:
                #     ok = comms.send_command(home_cmd1, home_cmd2, 0)
                #     if not ok:
                #         log.warning("  Arms-home command failed — continuing to tuck.")
                #     elif not comms.wait_for_done("DONE", timeout=cfg["arms_in_timeout"]):
                #         log.warning("  Arms-home timed out (%.1fs).", cfg["arms_in_timeout"])
                # else:
                #     log.info("  [SIM] Arms-home command not sent.")

                # Step 2: move to configured tuck angles
                log.info("Bringing arms to tucked position …")
                cmd1, cmd2 = angles_to_commands(
                    cfg["arms_in_theta1_deg"], cfg["arms_in_theta2_deg"]
                )
                log.info("  arms-in: θ1=%.1f°  θ2=%.1f°  cmd=(%.2f, %.2f)",
                         cfg["arms_in_theta1_deg"], cfg["arms_in_theta2_deg"], cmd1, cmd2)
                if comms is not None:
                    ok = comms.send_command(cmd1, cmd2, 0)
                    if not ok:
                        log.error("  Arms-in command failed — continuing to rotate.")
                    elif not comms.wait_for_done("DONE", timeout=cfg["arms_in_timeout"]):
                        log.warning("  Arms-in timed out (%.1fs).", cfg["arms_in_timeout"])
                else:
                    log.info("  [SIM] Arms-in command not sent.")
                state = State.ROTATE_BACK

            # ── ROTATE BACK ───────────────────────────────────────────────────
            elif state == State.ROTATE_BACK:
                log.info("Rotating stepper back to pickup position …")
                if comms is not None:
                    ok = comms.send_rotate()
                    if not ok:
                        log.error("  ROTATE command failed — continuing anyway.")
                    elif not comms.wait_for_done("DONE", timeout=cfg["rotate_timeout"]):
                        log.warning("  Rotate timed out waiting for DONE (%.1fs).",
                                    cfg["rotate_timeout"])
                else:
                    log.info("  [SIM] ROTATE command not sent.")

                ok = countdown(cfg["rotate_settle"], "Settling",
                               cap, cfg["show_preview"], H, H_inv,
                               vcfg, K, dist, last_detections, comms)
                if not ok:
                    break

                state = State.MOVE_TO_OBJ

            # ── MOVE TO OBJECT ────────────────────────────────────────────────
            elif state == State.MOVE_TO_OBJ:
                visited = total_objs - len(visit_list)
                log.info("── Object %d / %d  (%s) ─────────────────────",
                         visited, total_objs, current["color"])

                x_mm = current["x_mm"]
                y_mm = current["y_mm"]

                # Apply quadrant correction for +X / +Y region
                if x_mm > 0.0 and y_mm > 0.0:
                    off_x = cfg["quadrant_pp_offset_x_mm"]
                    off_y = cfg["quadrant_pp_offset_y_mm"]
                    if off_x != 0.0 or off_y != 0.0:
                        log.info("  [quadrant offset] raw=(%.1f, %.1f)  "
                                 "offset=(%.1f, %.1f)  corrected=(%.1f, %.1f)",
                                 x_mm, y_mm, off_x, off_y,
                                 x_mm + off_x, y_mm + off_y)
                    x_mm += off_x
                    y_mm += off_y

                ok = move_to(
                    x_mm, y_mm,
                    comms,
                    label=f"OBJ-{visited}",
                    drain_timeout=cfg["move_drain_timeout"],
                    object_angle_deg=current["angle_deg"],
                )

                if not ok:
                    log.warning("Skipping object %d (IK failed).", visited)
                    if visit_list:
                        current = visit_list.pop(0)
                        # stay in MOVE_TO_OBJ
                    else:
                        state = State.DONE
                else:
                    state = State.PICK_OBJ

            # ── PICK OBJECT ───────────────────────────────────────────────────
            elif state == State.PICK_OBJ:
                log.info("  Picking up object …")
                if comms is not None:
                    ok = comms.send_pick()
                    if ok:
                        if not comms.wait_for_done("DONE", timeout=cfg["pick_timeout"]):
                            log.warning("  PICK timed out (%.1fs).", cfg["pick_timeout"])
                else:
                    log.info("  [SIM] PICK not sent.")
                state = State.ARMS_IN_AFTER_PICK

            # ── ARMS IN AFTER PICK ────────────────────────────────────────────
            elif state == State.ARMS_IN_AFTER_PICK:
                # Same tuck sequence as ARMS_IN, then go to ROTATE_TO_DROP
                # log.info("Moving arms to home position (90°, 90°) …")
                # home_cmd1, home_cmd2 = angles_to_commands(90.0, 90.0)
                # if comms is not None:
                #     ok = comms.send_command(home_cmd1, home_cmd2, 0)
                #     if not ok:
                #         log.warning("  Arms-home command failed — continuing to tuck.")
                #     elif not comms.wait_for_done("DONE", timeout=cfg["arms_in_timeout"]):
                #         log.warning("  Arms-home timed out (%.1fs).", cfg["arms_in_timeout"])
                # else:
                #     log.info("  [SIM] Arms-home command not sent.")

                log.info("Bringing arms to tucked position …")
                cmd1, cmd2 = angles_to_commands(
                    cfg["arms_in_theta1_deg"], cfg["arms_in_theta2_deg"]
                )
                if comms is not None:
                    ok = comms.send_command(cmd1, cmd2, 0)
                    if not ok:
                        log.error("  Arms-in command failed — continuing.")
                    elif not comms.wait_for_done("DONE", timeout=cfg["arms_in_timeout"]):
                        log.warning("  Arms-in timed out (%.1fs).", cfg["arms_in_timeout"])
                else:
                    log.info("  [SIM] Arms-in command not sent.")
                state = State.ROTATE_TO_DROP

            # ── ROTATE TO DROP ────────────────────────────────────────────────
            elif state == State.ROTATE_TO_DROP:
                log.info("Rotating stepper back to drop zone …")
                if comms is not None:
                    ok = comms.send_rotate_home()
                    if not ok:
                        log.error("  ROTATE_HOME command failed — continuing anyway.")
                    elif not comms.wait_for_done("DONE", timeout=cfg["rotate_home_timeout"]):
                        log.warning("  Rotate-home timed out (%.1fs).",
                                    cfg["rotate_home_timeout"])
                else:
                    log.info("  [SIM] ROTATE_HOME command not sent.")

                ok = countdown(cfg["rotate_home_settle"], "Settling",
                               cap, cfg["show_preview"], H, H_inv,
                               vcfg, K, dist, last_detections, comms)
                if not ok:
                    break
                state = State.MOVE_TO_DROP

            # ── MOVE TO DROP POSITION ─────────────────────────────────────────
            elif state == State.MOVE_TO_DROP:
                color  = current["color"]
                theta1 = cfg.get(f"drop_{color}_theta1_deg", cfg["drop_red_theta1_deg"])
                theta2 = cfg.get(f"drop_{color}_theta2_deg", cfg["drop_red_theta2_deg"])
                log.info("Moving to %s drop position: θ1=%.1f°  θ2=%.1f°",
                         color, theta1, theta2)
                cmd1, cmd2 = angles_to_commands(theta1, theta2)
                log.info("  drop-move: cmd=(%.2f, %.2f)", cmd1, cmd2)
                if comms is not None:
                    ok = comms.send_command(cmd1, cmd2, 0)
                    if not ok:
                        log.error("  Drop-move command failed.")
                    elif not comms.wait_for_done("DONE", timeout=cfg["move_drain_timeout"]):
                        log.warning("  Drop-move timed out (%.1fs).", cfg["move_drain_timeout"])
                else:
                    log.info("  [SIM] Drop-move command not sent.")
                state = State.DROP_OBJ

            # ── DROP OBJECT ───────────────────────────────────────────────────
            elif state == State.DROP_OBJ:
                log.info("  Dropping object …")
                if comms is not None:
                    ok = comms.send_drop()
                    if ok:
                        if not comms.wait_for_done("DONE", timeout=cfg["drop_timeout"]):
                            log.warning("  DROP timed out (%.1fs).", cfg["drop_timeout"])
                else:
                    log.info("  [SIM] DROP not sent.")

                if visit_list:
                    current = visit_list.pop(0)
                    state   = State.ARMS_IN
                else:
                    state = State.RESCAN

            # ── RESCAN ────────────────────────────────────────────────────────
            elif state == State.RESCAN:
                log.info("All queued objects done — dwelling %.0fs before re-scan …",
                         cfg["rescan_dwell"])
                ok = countdown(cfg["rescan_dwell"], "Re-scan in",
                               cap, cfg["show_preview"], H, H_inv,
                               vcfg, K, dist, last_detections, comms)
                if not ok:
                    break

                log.info("Re-scanning for new objects …")
                ret, frame = cap.read()
                if not ret:
                    log.warning("Failed to capture re-scan frame — stopping.")
                    state = State.DONE
                    continue

                if K is not None:
                    frame = cv2.undistort(frame, K, dist)

                new_dets    = get_detections(frame, H, vcfg)
                new_ordered = order_picks(new_dets, cfg["pick_order"])
                last_detections = list(new_ordered)

                if not new_ordered:
                    log.info("Re-scan: no new objects found — run complete.")
                    state = State.DONE
                    continue

                visit_list  = new_ordered
                total_objs += len(new_ordered)
                log.info("Re-scan found %d new object(s):", len(new_ordered))
                for i, d in enumerate(new_ordered):
                    log.info("  %d. %s  (%.1f, %.1f) mm  angle=%.1f°",
                             i + 1, d["color"], d["x_mm"], d["y_mm"], d["angle_deg"])

                current = visit_list.pop(0)
                state   = State.ARMS_IN

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