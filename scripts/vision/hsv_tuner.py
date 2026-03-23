"""
HSV Mask Tuner for Benchy Detection
------------------------------------
Interactive trackbar tool to dial in red and blue HSV ranges
before building the full detection pipeline.

Usage:
    python hsv_tuner.py                      # Use webcam (index 0)
    python hsv_tuner.py --source image.jpg   # Use a static image
    python hsv_tuner.py --source video.mp4   # Use a video file
    python hsv_tuner.py --camera 1           # Use a different camera index

Controls:
    TAB       - Cycle active color (Red / Blue)
    S         - Save current HSV ranges to hsv_config.json
    L         - Load HSV ranges from hsv_config.json
    SPACE     - Pause/resume (video/camera mode)
    Q / ESC   - Quit

Output windows:
    [Original]  - Undistorted raw frame with detected contour overlays
    [Mask]      - Combined binary mask for active color
    [Tuner]     - Trackbar controls
"""

import cv2
import numpy as np
import json
import argparse
import os
import sys
from pathlib import Path

# ── Default HSV ranges ────────────────────────────────────────────────────────
# OpenCV HSV: H=[0,180], S=[0,255], V=[0,255]
# Red wraps around 0 so we use two sub-ranges and OR them.

DEFAULT_CONFIG = {
    "red": {
        "low1":  [0,   100, 60],   # lower red hue band
        "high1": [10,  255, 255],
        "low2":  [165, 100, 60],   # upper red hue band (wraps at 180)
        "high2": [180, 255, 255],
    },
    "blue": {
        "low":  [100, 80,  60],
        "high": [130, 255, 255],
    },
}

CONFIG_PATH = Path("hsv_config.json")

# ── Contour filtering parameters ──────────────────────────────────────────────
MIN_AREA    = 200    # px² — reject tiny noise blobs
MAX_AREA    = 80000  # px² — reject full-frame false positives
MIN_SOLIDITY = 0.25  # contour area / convex hull area — rejects spindly shapes

# ── Morphological kernel ──────────────────────────────────────────────────────
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


# ─────────────────────────────────────────────────────────────────────────────
# Trackbar helpers
# ─────────────────────────────────────────────────────────────────────────────

def nothing(_):
    pass


def make_trackbars(win: str, color: str, config: dict):
    """Create trackbars in `win` for the given color's HSV ranges."""
    try:
        cv2.destroyWindow(win)
    except cv2.error:
        pass
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 500, 300 if color == "red" else 200)

    if color == "red":
        r = config["red"]
        cv2.createTrackbar("H low1",  win, r["low1"][0],  180, nothing)
        cv2.createTrackbar("H high1", win, r["high1"][0], 180, nothing)
        cv2.createTrackbar("H low2",  win, r["low2"][0],  180, nothing)
        cv2.createTrackbar("H high2", win, r["high2"][0], 180, nothing)
        cv2.createTrackbar("S min",   win, r["low1"][1],  255, nothing)
        cv2.createTrackbar("V min",   win, r["low1"][2],  255, nothing)
    else:
        b = config["blue"]
        cv2.createTrackbar("H low",  win, b["low"][0],  180, nothing)
        cv2.createTrackbar("H high", win, b["high"][0], 180, nothing)
        cv2.createTrackbar("S min",  win, b["low"][1],  255, nothing)
        cv2.createTrackbar("V min",  win, b["low"][2],  255, nothing)


def read_trackbars(win: str, color: str) -> dict:
    """Read current trackbar values and return updated config slice."""
    g = lambda n: cv2.getTrackbarPos(n, win)
    if color == "red":
        s_min = g("S min")
        v_min = g("V min")
        return {
            "low1":  [g("H low1"),  s_min, v_min],
            "high1": [g("H high1"), 255,   255],
            "low2":  [g("H low2"),  s_min, v_min],
            "high2": [g("H high2"), 255,   255],
        }
    else:
        s_min = g("S min")
        v_min = g("V min")
        return {
            "low":  [g("H low"),  s_min, v_min],
            "high": [g("H high"), 255,   255],
        }


# ─────────────────────────────────────────────────────────────────────────────
# Mask builders
# ─────────────────────────────────────────────────────────────────────────────

def build_red_mask(hsv: np.ndarray, cfg: dict) -> np.ndarray:
    m1 = cv2.inRange(hsv, np.array(cfg["low1"]), np.array(cfg["high1"]))
    m2 = cv2.inRange(hsv, np.array(cfg["low2"]), np.array(cfg["high2"]))
    mask = cv2.bitwise_or(m1, m2)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)


def build_blue_mask(hsv: np.ndarray, cfg: dict) -> np.ndarray:
    mask = cv2.inRange(hsv, np.array(cfg["low"]), np.array(cfg["high"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    return mask



# ─────────────────────────────────────────────────────────────────────────────
# Contour analysis
# ─────────────────────────────────────────────────────────────────────────────

def filter_contours(contours):
    """Return contours that pass area and solidity thresholds."""
    valid = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (MIN_AREA < area < MAX_AREA):
            continue
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area == 0:
            continue
        if area / hull_area >= MIN_SOLIDITY:
            valid.append(c)
    return valid


def get_centroid(c) -> tuple[int, int]:
    M = cv2.moments(c)
    if M["m00"] == 0:
        return (0, 0)
    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


def draw_detections(frame: np.ndarray, mask: np.ndarray, color_bgr: tuple, label: str):
    """Find contours in mask, draw overlays on frame, print detection info."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = filter_contours(contours)

    for i, c in enumerate(valid):
        cx, cy   = get_centroid(c)
        rect     = cv2.minAreaRect(c)
        angle    = rect[2]          # degrees, OpenCV convention
        box_pts  = cv2.boxPoints(rect).astype(np.int32)

        # Draw rotated bounding box and centroid
        cv2.drawContours(frame, [box_pts], 0, color_bgr, 2)
        cv2.circle(frame, (cx, cy), 5, color_bgr, -1)

        # Orientation line through centroid
        length = 40
        rad = np.deg2rad(angle)
        x2 = int(cx + length * np.cos(rad))
        y2 = int(cy + length * np.sin(rad))
        cv2.line(frame, (cx, cy), (x2, y2), (255, 255, 255), 2)

        # Label
        tag = f"{label} #{i+1}  ({cx},{cy})  {angle:.1f}deg"
        cv2.putText(frame, tag, (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_bgr, 1, cv2.LINE_AA)

    return len(valid)


# ─────────────────────────────────────────────────────────────────────────────
# Config persistence
# ─────────────────────────────────────────────────────────────────────────────

def save_config(config: dict):
    with open(CONFIG_PATH, "w") as f:
        json.dump(config, f, indent=2)
    print(f"[HSV Tuner] Config saved to {CONFIG_PATH}")


def load_config() -> dict:
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
        print(f"[HSV Tuner] Config loaded from {CONFIG_PATH}")
        return cfg
    print("[HSV Tuner] No saved config found, using defaults.")
    return DEFAULT_CONFIG.copy()


# ─────────────────────────────────────────────────────────────────────────────
# Camera intrinsics (optional undistortion)
# ─────────────────────────────────────────────────────────────────────────────

def load_intrinsics():
    cam_path  = Path("intrinsics/camera_matrix.npy")
    dist_path = Path("intrinsics/dist_coeffs.npy")
    if cam_path.exists() and dist_path.exists():
        K    = np.load(str(cam_path))
        dist = np.load(str(dist_path))
        print("[HSV Tuner] Camera intrinsics loaded — undistortion active.")
        return K, dist
    print("[HSV Tuner] No intrinsics found — skipping undistortion.")
    return None, None


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="HSV Mask Tuner for Benchy Detection")
    parser.add_argument("--source", type=str, default=None,
                        help="Image or video path. Omit to use webcam.")
    parser.add_argument("--camera", type=int, default=0,
                        help="Webcam device index (default 0).")
    args = parser.parse_args()

    # ── Source setup ──────────────────────────────────────────────────────────
    is_image = False
    if args.source and Path(args.source).suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"):
        static_frame = cv2.imread(args.source)
        if static_frame is None:
            sys.exit(f"[HSV Tuner] Cannot read image: {args.source}")
        is_image = True
        cap = None
    else:
        src = args.source if args.source else args.camera
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            sys.exit(f"[HSV Tuner] Cannot open source: {src}")

    # ── Intrinsics ────────────────────────────────────────────────────────────
    K, dist = load_intrinsics()

    # ── State ─────────────────────────────────────────────────────────────────
    config       = load_config()
    active_color = "red"   # TAB toggles between red / blue
    paused       = False
    frame        = None

    TUNER_WIN  = "[ Tuner ]"
    ORIG_WIN   = "[ Original ]"
    MASK_WIN   = "[ Mask ]"

    cv2.namedWindow(ORIG_WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WIN, cv2.WINDOW_NORMAL)
    make_trackbars(TUNER_WIN, active_color, config)

    print("\n[HSV Tuner] Ready.")
    print("  TAB   — toggle Red / Blue")
    print("  S     — save config")
    print("  L     — load config")
    print("  SPACE — pause/resume")
    print("  Q/ESC — quit\n")

    while True:
        # ── Grab frame ────────────────────────────────────────────────────────
        if is_image:
            frame = static_frame.copy()
        elif not paused:
            ret, frame = cap.read()
            if not ret:
                print("[HSV Tuner] End of source.")
                break

        if frame is None:
            continue

        # ── Undistort ─────────────────────────────────────────────────────────
        if K is not None:
            frame = cv2.undistort(frame, K, dist)

        # ── Read trackbars → update config ────────────────────────────────────
        config[active_color] = read_trackbars(TUNER_WIN, active_color)

        # ── Build masks ───────────────────────────────────────────────────────
        hsv        = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask   = build_red_mask(hsv,  config["red"])
        blue_mask  = build_blue_mask(hsv, config["blue"])

        display    = frame.copy()
        n_red      = draw_detections(display, red_mask,  (40,  40,  220), "Red")
        n_blue     = draw_detections(display, blue_mask, (200, 100, 20),  "Blue")

        # ── Active mask for the mask window ───────────────────────────────────
        active_mask = red_mask if active_color == "red" else blue_mask

        # ── HUD overlay ───────────────────────────────────────────────────────
        hud = f"Tuning: {active_color.upper()}  |  Red: {n_red}  Blue: {n_blue}  |  TAB=switch  S=save  Q=quit"
        cv2.putText(display, hud, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA)

        # ── Show ──────────────────────────────────────────────────────────────
        cv2.imshow(ORIG_WIN, display)
        cv2.imshow(MASK_WIN, active_mask)

        # ── Key handling ──────────────────────────────────────────────────────
        key = cv2.waitKey(1 if not is_image else 30) & 0xFF

        if key in (ord('q'), 27):           # Q or ESC
            break
        elif key == 9:                      # TAB — toggle color
            active_color = "blue" if active_color == "red" else "red"
            make_trackbars(TUNER_WIN, active_color, config)
            print(f"[HSV Tuner] Switched to: {active_color.upper()}")
        elif key == ord('s'):               # S — save
            save_config(config)
        elif key == ord('l'):               # L — load
            config = load_config()
            make_trackbars(TUNER_WIN, active_color, config)
        elif key == ord(' '):               # SPACE — pause
            paused = not paused
            print(f"[HSV Tuner] {'Paused' if paused else 'Resumed'}")

    if cap:
        cap.release()
    cv2.destroyAllWindows()

    print("\n[HSV Tuner] Final config:")
    print(json.dumps(config, indent=2))


if __name__ == "__main__":
    main()