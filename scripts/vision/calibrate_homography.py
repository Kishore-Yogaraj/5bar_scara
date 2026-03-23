"""
Homography Calibration Tool
-----------------------------
Click N points on an input image, enter the corresponding real-world
(robot-frame) coordinates in mm for each point, then compute and save
the homography matrix to a JSON file.

Usage:
    python calibrate_homography.py --image <path> [--points 13] [--output homography.json]

Controls (during point selection):
    Left-click      — place a point
    Z               — undo last point
    Q / ESC         — quit without saving
"""

import cv2
import numpy as np
import json
import argparse
import sys
from pathlib import Path


# ── Display constants ──────────────────────────────────────────────────────────
POINT_COLOR     = (0,   255, 100)   # green dots for placed points
ACTIVE_COLOR    = (0,   200, 255)   # yellow-ish crosshair for next point
TEXT_COLOR      = (255, 255, 255)
FONT            = cv2.FONT_HERSHEY_SIMPLEX
POINT_RADIUS    = 6
CROSS_SIZE      = 20

WINDOW_NAME     = "[ Homography Calibration ]"


# ── State ─────────────────────────────────────────────────────────────────────
pixel_points    = []   # list of (u, v)
world_points    = []   # list of (X_mm, Y_mm)
mouse_pos       = [0, 0]


def mouse_callback(event, x, y, flags, param):
    mouse_pos[0], mouse_pos[1] = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        param["clicked"] = (x, y)


def draw_overlay(base: np.ndarray, n_total: int) -> np.ndarray:
    """Draw all placed points, index labels, crosshair, and status HUD."""
    frame = base.copy()
    h, w  = frame.shape[:2]

    # ── Placed points ────────────────────────────────────────────────────────
    for i, (u, v) in enumerate(pixel_points):
        cv2.circle(frame, (u, v), POINT_RADIUS, POINT_COLOR, -1)
        cv2.circle(frame, (u, v), POINT_RADIUS + 2, (0, 0, 0), 1)
        label = f"#{i+1}"
        cv2.putText(frame, label, (u + 10, v - 8), FONT, 0.45,
                    (0, 0, 0),    2, cv2.LINE_AA)
        cv2.putText(frame, label, (u + 10, v - 8), FONT, 0.45,
                    POINT_COLOR,  1, cv2.LINE_AA)

    # ── Live crosshair for next point ────────────────────────────────────────
    # n_placed = len(pixel_points)
    # if n_placed < n_total:
    #     mx, my = mouse_pos
    #     cv2.line(frame, (mx - CROSS_SIZE, my), (mx + CROSS_SIZE, my), ACTIVE_COLOR, 1)
    #     cv2.line(frame, (mx, my - CROSS_SIZE), (mx, my + CROSS_SIZE), ACTIVE_COLOR, 1)
    #     cv2.circle(frame, (mx, my), 3, ACTIVE_COLOR, -1)

    # # ── HUD bar at top ────────────────────────────────────────────────────────
    # cv2.rectangle(frame, (0, 0), (w, 36), (20, 20, 20), -1)
    # if n_placed < n_total:
    #     status = f"Point {n_placed + 1} / {n_total}  |  Left-click to place  |  Z = undo  |  Q = quit"
    # else:
    #     status = f"All {n_total} points placed. Close this window or press Q to finish."
    # cv2.putText(frame, status, (10, 24), FONT, 0.5, TEXT_COLOR, 1, cv2.LINE_AA)

    return frame


def collect_pixel_points(image: np.ndarray, n_total: int) -> bool:
    """
    Show the image in a window and let the user click n_total points.
    Returns True when done, False if the user quit early.
    """
    global pixel_points, mouse_pos

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    click_state = {"clicked": None}
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback, click_state)

    # Fit window to screen without making it tiny
    h, w = image.shape[:2]
    screen_w, screen_h = 1600, 900  # conservative default
    scale = min(screen_w / w, screen_h / h, 1.0)
    cv2.resizeWindow(WINDOW_NAME, int(w * scale), int(h * scale))

    while True:
        display = draw_overlay(image, n_total)
        cv2.imshow(WINDOW_NAME, display)
        key = cv2.waitKey(30) & 0xFF

        # Check for a click
        if click_state["clicked"] is not None and len(pixel_points) < n_total:
            pixel_points.append(click_state["clicked"])
            print(f"\n  ✓ Pixel point #{len(pixel_points)}: {click_state['clicked']}")
            click_state["clicked"] = None

        # Undo
        if key == ord('z') and pixel_points:
            removed = pixel_points.pop()
            if world_points:
                world_points.pop()
            print(f"  ↩ Undid point {len(pixel_points) + 1} (pixel {removed})")

        # Quit
        if key in (ord('q'), 27):
            cv2.destroyAllWindows()
            return False

        # Done collecting — exit loop so terminal prompts can begin
        if len(pixel_points) == n_total:
            # Pause briefly so user sees all points placed before terminal takes over
            cv2.waitKey(400)
            break

    cv2.destroyAllWindows()
    return True


def prompt_world_points(n_total: int):
    """
    For each pixel point already collected, prompt the user to type
    the real-world X and Y in mm.
    """
    print("\n" + "═" * 58)
    print("  Enter real-world coordinates (mm) for each pixel point.")
    print("  Robot frame: X = right, Y = forward, origin = robot center")
    print("═" * 58 + "\n")

    for i, (u, v) in enumerate(pixel_points):
        print(f"  Point #{i+1}  ←  pixel ({u}, {v})")
        while True:
            try:
                x_str = input(f"    X (mm): ").strip()
                y_str = input(f"    Y (mm): ").strip()
                x_mm  = float(x_str)
                y_mm  = float(y_str)
                world_points.append((x_mm, y_mm))
                print(f"    → Robot frame: ({x_mm:.2f}, {y_mm:.2f}) mm\n")
                break
            except ValueError:
                print("    ✗ Invalid input — please enter a number (e.g. 150.0)\n")


def compute_and_save(output_path: Path, image_shape: tuple):
    """Compute homography from correspondences and write JSON."""
    src = np.array(pixel_points,  dtype=np.float64)   # pixel  (u, v)
    dst = np.array(world_points,  dtype=np.float64)   # world  (X, Y) mm

    # findHomography expects (N,1,2) shape
    H, mask = cv2.findHomography(
        src.reshape(-1, 1, 2),
        dst.reshape(-1, 1, 2),
        cv2.RANSAC,
        ransacReprojThreshold=5.0   # mm — tune if needed
    )

    if H is None:
        print("\n✗ Homography computation failed. Check that points are not collinear.")
        sys.exit(1)

    inliers = int(mask.sum())
    print(f"\n  Homography computed.  Inliers: {inliers} / {len(pixel_points)}")

    # ── Reprojection error ────────────────────────────────────────────────────
    errors = []
    for (u, v), (Xw, Yw) in zip(pixel_points, world_points):
        pt_h = H @ np.array([u, v, 1.0])
        Xp, Yp = pt_h[0] / pt_h[2], pt_h[1] / pt_h[2]
        err = np.hypot(Xp - Xw, Yp - Yw)
        errors.append(float(err))

    mean_err = float(np.mean(errors))
    max_err  = float(np.max(errors))
    print(f"  Reprojection error  mean: {mean_err:.3f} mm   max: {max_err:.3f} mm")

    # ── Build JSON payload ────────────────────────────────────────────────────
    payload = {
        "metadata": {
            "image_resolution": [image_shape[1], image_shape[0]],   # [width, height]
            "num_points": len(pixel_points),
            "inliers": inliers,
            "reprojection_error_mm": {
                "mean": round(mean_err, 4),
                "max":  round(max_err,  4),
                "per_point": [round(e, 4) for e in errors],
            },
        },
        "correspondences": [
            {
                "id":    i + 1,
                "pixel": list(pixel_points[i]),
                "world_mm": list(world_points[i]),
                "reprojection_error_mm": round(errors[i], 4),
            }
            for i in range(len(pixel_points))
        ],
        # Row-major 3×3 homography matrix.
        # Usage: pt_h = H @ [u, v, 1]; X = pt_h[0]/pt_h[2]; Y = pt_h[1]/pt_h[2]
        "homography_matrix": H.tolist(),
    }

    with open(output_path, "w") as f:
        json.dump(payload, f, indent=2)

    print(f"\n  ✓ Saved to: {output_path}\n")
    print("  Homography matrix (H):")
    for row in H:
        print("   ", "  ".join(f"{v:12.6f}" for v in row))
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Homography Calibration Tool")
    parser.add_argument("--image",   required=True,           help="Path to calibration image")
    parser.add_argument("--points",  type=int, default=13,    help="Number of calibration points (default 13)")
    parser.add_argument("--output",  default="homography.json", help="Output JSON path (default: homography.json)")
    args = parser.parse_args()

    # ── Load image ────────────────────────────────────────────────────────────
    img_path = Path(args.image)
    if not img_path.exists():
        sys.exit(f"✗ Image not found: {img_path}")

    image = cv2.imread(str(img_path))
    if image is None:
        sys.exit(f"✗ Could not read image: {img_path}")

    n = args.points
    print(f"\n[Homography Calibration]")
    print(f"  Image   : {img_path}  ({image.shape[1]}×{image.shape[0]})")
    print(f"  Points  : {n}")
    print(f"  Output  : {args.output}\n")
    print("  Click each calibration point on the image.")
    print("  After all points are placed, you will be prompted for real-world coords.\n")

    # ── Phase 1: Collect pixel coords via clicking ─────────────────────────────
    ok = collect_pixel_points(image, n)
    if not ok or len(pixel_points) < 4:
        sys.exit("✗ Aborted — need at least 4 points for a valid homography.")

    # ── Phase 2: Collect world coords via terminal ────────────────────────────
    prompt_world_points(n)

    # ── Phase 3: Compute and save ─────────────────────────────────────────────
    compute_and_save(Path(args.output), image.shape)


if __name__ == "__main__":
    main()