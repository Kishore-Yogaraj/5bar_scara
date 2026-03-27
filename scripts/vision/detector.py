"""
detector.py  —  Vision detection library for the 5-bar SCARA robot.

Detects coloured objects (red / blue) via HSV thresholding, transforms each
centroid into the robot frame (mm) via a precomputed homography, and returns
structured detection data that pick_and_place.py can consume directly.

Public API
----------
load_homography(path)           -> np.ndarray (3×3)
load_hsv_config(path)           -> dict
load_intrinsics(folder)         -> (K, dist) | (None, None)

get_detections(frame, H, cfg)   -> list[Detection]
    Each Detection is a dict:
        color     : "red" | "blue"
        x_mm      : float  — robot-frame X (mm)
        y_mm      : float  — robot-frame Y (mm)
        angle_deg : float  — short-axis orientation from robot +X (°, CCW+)
        cx        : int    — pixel centroid u
        cy        : int    — pixel centroid v
        area      : float  — contour area (px²)

annotate_frame(frame, detections, H, H_inv) -> np.ndarray
    Draw bounding boxes, centroids, orientation arrows, and world-frame
    labels onto a copy of *frame*.  Use for debugging / live preview.

Standalone usage
----------------
    python detector.py                          # webcam index 0
    python detector.py --camera 1
    python detector.py --source image.jpg
    python detector.py --homography cal.json
    python detector.py --hsv hsv_config.json

Controls
--------
    Q / ESC  —  quit
"""

from __future__ import annotations

import cv2
import numpy as np
import json
import argparse
import sys
import logging
from pathlib import Path
from typing import TypedDict

log = logging.getLogger(__name__)

# ── Contour filtering ─────────────────────────────────────────────────────────
MIN_AREA     = 200
MAX_AREA     = 80_000
MIN_SOLIDITY = 0.25

# ── Morphological kernels ─────────────────────────────────────────────────────
_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,  5))
_CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

FONT = cv2.FONT_HERSHEY_SIMPLEX


# ─────────────────────────────────────────────────────────────────────────────
# Detection data type
# ─────────────────────────────────────────────────────────────────────────────

class Detection(TypedDict):
    color     : str          # "red" | "blue"
    x_mm      : float        # robot-frame X (mm)
    y_mm      : float        # robot-frame Y (mm)
    angle_deg : float        # short-axis orientation from robot +X (°, CCW positive)
    cx        : int          # pixel centroid u
    cy        : int          # pixel centroid v
    area      : float        # contour area (px²) — useful for pick priority sorting
    box_pts   : np.ndarray   # (4, 2) int32 — rotated bounding box corners for drawing


# ─────────────────────────────────────────────────────────────────────────────
# I/O helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_homography(path: str | Path) -> np.ndarray:
    """
    Load a homography matrix from a JSON file produced by calibrate_homography.py.

    Expected JSON structure:
        {
          "homography_matrix": [[...], [...], [...]],
          "metadata": {
            "image_resolution": [w, h],
            "reprojection_error_mm": {"mean": ..., ...}
          }
        }
    """
    path = Path(path)
    with open(path) as f:
        data = json.load(f)

    H   = np.array(data["homography_matrix"], dtype=np.float64)
    res = data["metadata"]["image_resolution"]
    err = data["metadata"]["reprojection_error_mm"]["mean"]
    log.info(
        "Homography loaded from '%s'  (calibrated at %d×%d, mean reproj error %.3f mm)",
        path, res[0], res[1], err,
    )
    return H


def load_hsv_config(path: str | Path) -> dict:
    """Load HSV threshold ranges from a JSON file produced by hsv_tuner.py."""
    path = Path(path)
    with open(path) as f:
        cfg = json.load(f)
    log.info("HSV config loaded from '%s'.", path)
    return cfg


def load_intrinsics(folder: str | Path = "intrinsics") -> tuple[np.ndarray | None, np.ndarray | None]:
    """
    Load camera matrix K and distortion coefficients from .npy files.

    Returns (K, dist) if both files exist, otherwise (None, None) — callers
    should skip undistortion when None is returned.
    """
    folder    = Path(folder)
    cam_path  = folder / "camera_matrix.npy"
    dist_path = folder / "dist_coeffs.npy"

    if cam_path.exists() and dist_path.exists():
        K    = np.load(str(cam_path))
        dist = np.load(str(dist_path))
        log.info("Camera intrinsics loaded from '%s' — undistortion active.", folder)
        return K, dist

    log.warning("Intrinsics not found in '%s' — skipping undistortion.", folder)
    return None, None


# ─────────────────────────────────────────────────────────────────────────────
# Coordinate transforms
# ─────────────────────────────────────────────────────────────────────────────

def _pixel_to_world(H: np.ndarray, u: float, v: float) -> tuple[float, float]:
    """Map a single pixel (u, v) → robot-frame (X_mm, Y_mm)."""
    pt = H @ np.array([u, v, 1.0], dtype=np.float64)
    return pt[0] / pt[2], pt[1] / pt[2]


def _world_to_pixel(H_inv: np.ndarray, X: float, Y: float) -> tuple[int, int]:
    """Map a robot-frame point (X_mm, Y_mm) → pixel (u, v)."""
    pt = H_inv @ np.array([X, Y, 1.0], dtype=np.float64)
    return int(pt[0] / pt[2]), int(pt[1] / pt[2])


def _homography_jacobian(H: np.ndarray, u: float, v: float) -> np.ndarray:
    """
    2×2 Jacobian of the homography projection at pixel (u, v).

    Gives the local linear map from pixel-space direction vectors to
    robot-frame direction vectors, correcting for perspective distortion.
    """
    w    = H[2, 0] * u + H[2, 1] * v + H[2, 2]
    num0 = H[0, 0] * u + H[0, 1] * v + H[0, 2]
    num1 = H[1, 0] * u + H[1, 1] * v + H[1, 2]
    w2   = w * w

    dXdu = (H[0, 0] * w - num0 * H[2, 0]) / w2
    dXdv = (H[0, 1] * w - num0 * H[2, 1]) / w2
    dYdu = (H[1, 0] * w - num1 * H[2, 0]) / w2
    dYdv = (H[1, 1] * w - num1 * H[2, 1]) / w2

    return np.array([[dXdu, dXdv], [dYdu, dYdv]], dtype=np.float64)


def _parallax_correct(
    u: float,
    v: float,
    K: np.ndarray,
    z_obj_mm: float,
) -> tuple[float, float]:
    """
    Shift a detected top-surface centroid toward the base-centre position to
    remove parallax offset caused by object height.

    Uses the pinhole small-angle approximation:
        u_corr = u - (u - cx) * z_obj_mm / fx
        v_corr = v - (v - cy) * z_obj_mm / fy

    The correction is zero at the principal point and grows linearly with
    distance from centre, exactly compensating for the perspective shift of a
    flat top surface at height z_obj_mm above the calibration plane.
    """
    cx_px = K[0, 2];  cy_px = K[1, 2]
    fx    = K[0, 0];  fy    = K[1, 1]
    return (u - (u - cx_px) * z_obj_mm / fx,
            v - (v - cy_px) * z_obj_mm / fy)


# ─────────────────────────────────────────────────────────────────────────────
# Mask builders
# ─────────────────────────────────────────────────────────────────────────────

def _build_red_mask(hsv: np.ndarray, cfg: dict) -> np.ndarray:
    r  = cfg["red"]
    m1 = cv2.inRange(hsv, np.array(r["low1"]), np.array(r["high1"]))
    m2 = cv2.inRange(hsv, np.array(r["low2"]), np.array(r["high2"]))
    return cv2.morphologyEx(cv2.bitwise_or(m1, m2), cv2.MORPH_OPEN, _MORPH_KERNEL)


def _build_blue_mask(hsv: np.ndarray, cfg: dict) -> np.ndarray:
    b    = cfg["blue"]
    mask = cv2.inRange(hsv, np.array(b["low"]), np.array(b["high"]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL)
    return mask


# ─────────────────────────────────────────────────────────────────────────────
# Contour helpers
# ─────────────────────────────────────────────────────────────────────────────

def _filter_contours(contours) -> list:
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


def _get_centroid(c) -> tuple[int, int]:
    M = cv2.moments(c)
    if M["m00"] == 0:
        return 0, 0
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


def _get_orientation(
    contour,
    H: np.ndarray,
    cx: int,
    cy: int,
) -> tuple[float, tuple[int, int]]:
    """
    Return the short-axis orientation angle (degrees, robot frame, CCW+) and
    a pixel tip point for drawing the orientation arrow.
    """
    _, (w, h), cv_angle_deg = cv2.minAreaRect(contour)

    rad   = np.deg2rad(cv_angle_deg)
    cos_a = np.cos(rad)
    sin_a = np.sin(rad)

    # Short-side pixel unit vector
    short_px = np.array([cos_a, sin_a]) if h <= w else np.array([-sin_a, cos_a])

    # Pick the candidate that points toward positive robot-frame Y
    J        = _homography_jacobian(H, float(cx), float(cy))
    robot_a  = J @  short_px
    robot_b  = J @ -short_px

    chosen_px    = short_px  if robot_a[1] >= 0.0 else -short_px
    chosen_robot = robot_a   if robot_a[1] >= 0.0 else  robot_b

    angle_deg = float(np.degrees(np.arctan2(chosen_robot[1], chosen_robot[0])))

    LINE_LEN  = 35
    tip       = (int(cx + LINE_LEN * chosen_px[0]), int(cy + LINE_LEN * chosen_px[1]))
    return angle_deg, tip


# ─────────────────────────────────────────────────────────────────────────────
# Core detection API
# ─────────────────────────────────────────────────────────────────────────────

def get_detections(
    frame: np.ndarray,
    H: np.ndarray,
    cfg: dict,
    K: np.ndarray | None = None,
    z_obj_mm: float = 0.0,
) -> list[Detection]:
    """
    Detect red and blue objects in *frame* and return their robot-frame poses.

    Parameters
    ----------
    frame : BGR image (as returned by cv2.imread / cap.read).
    H     : 3×3 homography matrix mapping pixels → robot mm.
    cfg   : HSV config dict (from load_hsv_config).

    Returns
    -------
    List of Detection dicts, each with:
        color, x_mm, y_mm, angle_deg, cx, cy, area

    The list is sorted by distance from the robot origin (nearest first) so
    pick_and_place.py can iterate straight through without reordering.
    """
    hsv       = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask  = _build_red_mask(hsv, cfg)
    blue_mask = _build_blue_mask(hsv, cfg)

    detections: list[Detection] = []

    for color, mask in (("red", red_mask), ("blue", blue_mask)):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in _filter_contours(contours):
            cx, cy        = _get_centroid(c)
            u, v          = float(cx), float(cy)
            if K is not None and z_obj_mm != 0.0:
                u, v = _parallax_correct(u, v, K, z_obj_mm)
            x_mm, y_mm    = _pixel_to_world(H, u, v)
            angle_deg, _  = _get_orientation(c, H, cx, cy)
            box_pts       = cv2.boxPoints(cv2.minAreaRect(c)).astype(np.int32)

            detections.append(Detection(
                color     = color,
                x_mm      = round(x_mm,  2),
                y_mm      = round(y_mm,  2),
                angle_deg = round(angle_deg, 2),
                cx        = cx,
                cy        = cy,
                area      = float(cv2.contourArea(c)),
                box_pts   = box_pts,
            ))

    # Sort nearest to origin first for default pick ordering
    detections.sort(key=lambda d: d["x_mm"] ** 2 + d["y_mm"] ** 2)
    return detections


# ─────────────────────────────────────────────────────────────────────────────
# Annotation helper  (debug / live preview only — not used by orchestrator)
# ─────────────────────────────────────────────────────────────────────────────

_COLOR_BGR = {"red": (40, 40, 220), "blue": (200, 100, 20)}


def annotate_frame(
    frame: np.ndarray,
    detections: list[Detection],
    H: np.ndarray,
    H_inv: np.ndarray,
) -> np.ndarray:
    """
    Draw bounding boxes, centroids, orientation arrows, and world-frame labels
    onto a copy of *frame*.

    Parameters
    ----------
    frame      : original BGR image.
    detections : output of get_detections().
    H          : homography (pixels → world).
    H_inv      : np.linalg.inv(H)  — used to project the origin crosshair.

    Returns
    -------
    Annotated BGR image (frame is not modified in-place).
    """
    display = frame.copy()

    # ── Robot-frame origin crosshair ──────────────────────────────────────────
    ox, oy = _world_to_pixel(H_inv, 0.0, 0.0)
    fh, fw = display.shape[:2]
    if -50 < ox < fw + 50 and -50 < oy < fh + 50:
        arm = 18
        cv2.line(display, (ox - arm, oy), (ox + arm, oy), (0,   0,   0), 3, cv2.LINE_AA)
        cv2.line(display, (ox, oy - arm), (ox, oy + arm), (0,   0,   0), 3, cv2.LINE_AA)
        cv2.line(display, (ox - arm, oy), (ox + arm, oy), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.line(display, (ox, oy - arm), (ox, oy + arm), (255, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(display, (ox, oy), 3, (255, 255, 255), -1, cv2.LINE_AA)
        for offset, col in ((2, (0, 0, 0)), (1, (255, 255, 255))):
            cv2.putText(display, "origin", (ox + 6, oy - 6),
                        FONT, 0.38, col, offset, cv2.LINE_AA)

    # ── Detections ────────────────────────────────────────────────────────────
    for color, bgr in _COLOR_BGR.items():
        color_dets = [d for d in detections if d["color"] == color]
        if not color_dets:
            continue

        for i, det in enumerate(color_dets):
            cx, cy = det["cx"], det["cy"]

            # Rotated bounding box
            cv2.drawContours(display, [det["box_pts"]], 0, bgr, 2, cv2.LINE_AA)
            cv2.circle(display, (cx, cy), 5, bgr, -1, cv2.LINE_AA)

            # Orientation arrow
            rad       = np.deg2rad(det["angle_deg"])
            J_inv     = np.linalg.inv(_homography_jacobian(H, float(cx), float(cy)))
            robot_dir = np.array([np.cos(rad), np.sin(rad)])
            px_dir    = J_inv @ robot_dir
            norm      = np.linalg.norm(px_dir)
            if norm > 1e-6:
                px_dir /= norm
            tip = (int(cx + 35 * px_dir[0]), int(cy + 35 * px_dir[1]))
            cv2.line(display, (cx, cy), tip, (255, 255, 255), 2, cv2.LINE_AA)

            # Label
            tag = (f"{color.capitalize()}#{i+1}"
                   f"({det['x_mm']:+.1f},{det['y_mm']:+.1f})"
                   f"{det['angle_deg']:+.1f}")
            tx, ty = cx + 10, cy - 10
            for offset, col in ((2, (0, 0, 0)), (1, bgr)):
                cv2.putText(display, tag, (tx, ty), FONT, 0.44, col, offset, cv2.LINE_AA)

    # ── HUD ───────────────────────────────────────────────────────────────────
    n_red  = sum(1 for d in detections if d["color"] == "red")
    n_blue = sum(1 for d in detections if d["color"] == "blue")
    hud    = f"Red: {n_red}  Blue: {n_blue}  |  Q = quit"
    for offset, col in ((2, (0, 0, 0)), (1, (220, 220, 220))):
        cv2.putText(display, hud, (8, 20), FONT, 0.48, col, offset, cv2.LINE_AA)

    return display


# ─────────────────────────────────────────────────────────────────────────────
# Standalone preview  (debug use only)
# ─────────────────────────────────────────────────────────────────────────────

def _run_preview(args: argparse.Namespace):
    """Live annotated preview — mirrors the old detect.py behaviour."""
    logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")

    H     = load_homography(args.homography)
    H_inv = np.linalg.inv(H)
    cfg   = load_hsv_config(args.hsv)
    K, dist = load_intrinsics(args.intrinsics)

    # Source setup
    is_image    = False
    cap         = None
    static_frame = None

    if args.source:
        ext = Path(args.source).suffix.lower()
        if ext in (".jpg", ".jpeg", ".png", ".bmp"):
            static_frame = cv2.imread(args.source)
            if static_frame is None:
                sys.exit(f"[Detector] Cannot read image: {args.source}")
            is_image = True
        else:
            cap = cv2.VideoCapture(args.source)
    else:
        cap = cv2.VideoCapture(args.camera)

    if not is_image and (cap is None or not cap.isOpened()):
        sys.exit("[Detector] Cannot open video source.")

    WIN = "[ Detector — debug preview ]"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    print("\n[Detector] Running.  Q / ESC to quit.\n")

    while True:
        if is_image:
            frame = static_frame.copy()
        else:
            ret, frame = cap.read()
            if not ret:
                print("[Detector] End of source.")
                break

        if K is not None:
            frame = cv2.undistort(frame, K, dist)

        dets = get_detections(frame, H, cfg, K=K, z_obj_mm=args.z_obj)

        # Print detections to console
        for d in dets:
            print(f"  {d['color']:4s}  ({d['x_mm']:+7.1f}, {d['y_mm']:+7.1f}) mm  "
                  f"angle={d['angle_deg']:+6.1f}°  area={d['area']:.0f}px²")
        if dets:
            print()

        display = annotate_frame(frame, dets, H, H_inv)
        cv2.imshow(WIN, display)

        key = cv2.waitKey(1 if not is_image else 30) & 0xFF
        if key in (ord('q'), 27):
            break

    if cap:
        cap.release()
    cv2.destroyAllWindows()
    print("[Detector] Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Detector — debug live preview")
    _here = Path(__file__).parent
    parser.add_argument("--source",     type=str, default=None)
    parser.add_argument("--camera",     type=int, default=0)
    parser.add_argument("--homography", type=str, default=str(_here / "homography.json"))
    parser.add_argument("--hsv",        type=str, default=str(_here / "hsv_config.json"))
    parser.add_argument("--intrinsics", type=str, default=str(_here / "intrinsics"))
    parser.add_argument("--z-obj",      type=float, default=0.0,
                        help="Object height above work surface (mm) for parallax correction")
    _run_preview(parser.parse_args())