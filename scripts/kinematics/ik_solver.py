"""
ik_solver.py  —  Pure kinematics library for the 5-bar parallel SCARA robot.

No matplotlib, no serial, no side effects.  Import this from any script that
needs IK, FK, or workspace computation.

Public API
----------
solve_ik(xp, yp, ...)              -> dict  {valid, theta1, theta2, ...}
forward_kinematics(theta1, theta2) -> dict | None  {M1, M2, E1, E2, P}
calculate_workspace(...)           -> np.ndarray (bool mask on a meshgrid)
angles_to_commands(theta1_deg, theta2_deg, ...) -> (cmd1, cmd2)
"""

import numpy as np

# ── Robot geometry constants ──────────────────────────────────────────────────
L1           = 200.0    # Proximal link length (mm)
L2           = 200.0    # Distal link length (mm)
MOTOR_DIST   = 100.0    # Distance between the two base motor pivots (mm)

# ── Operating constraints ─────────────────────────────────────────────────────
COMFORT_ANGLE_DEG = 15.0  # Minimum transmission angle from straight (deg).
                           # Points where |cos(elbow angle)| > cos(15°) are
                           # near-singular and rejected by the IK solver.

# ── Motor-to-IK mapping ───────────────────────────────────────────────────────
HOME_ANGLE = 90.0   # IK theta value (deg from +x) when the arm is at its
                    # physical home position (encoders = 0).

M1_SIGN = -1        # Flip to +1 / -1 to match the physical winding direction
M2_SIGN = -1        # of each motor relative to the IK convention.


# ─────────────────────────────────────────────────────────────────────────────
# Inverse Kinematics
# ─────────────────────────────────────────────────────────────────────────────

def solve_ik(
    xp: float,
    yp: float,
    L1: float               = L1,
    L2: float               = L2,
    motor_dist: float       = MOTOR_DIST,
    comfort_angle_deg: float = COMFORT_ANGLE_DEG,
) -> dict:
    """
    Solve IK for a target end-effector position (xp, yp) in the robot frame
    (mm).  Uses the elbows-out branch (left elbow left of target, right elbow
    right of target).

    Parameters
    ----------
    xp, yp            : target coordinates in mm.
    L1                : proximal link length (mm).
    L2                : distal link length (mm).
    motor_dist        : centre-to-centre distance between base pivots (mm).
    comfort_angle_deg : minimum allowed transmission angle (deg).  Points in
                        near-singular configurations are rejected.

    Returns
    -------
    dict with keys:
        valid       : bool — False if the point is unreachable / ill-conditioned.
        reason      : str  — human-readable failure reason (only when valid=False).
        x, y        : echo of the input target (mm).
        theta1      : left-motor proximal angle (radians, from +x axis).
        theta2      : right-motor proximal angle (radians, from +x axis).
        theta1_deg  : same, degrees.
        theta2_deg  : same, degrees.
    """
    L0 = motor_dist / 2.0

    D1 = np.hypot(xp + L0, yp)
    D2 = np.hypot(xp - L0, yp)

    # ── Basic reachability ────────────────────────────────────────────────────
    if not (abs(L1 - L2) <= D1 <= (L1 + L2)):
        return {"valid": False, "reason": "Left arm cannot reach target."}
    if not (abs(L1 - L2) <= D2 <= (L1 + L2)):
        return {"valid": False, "reason": "Right arm cannot reach target."}
    if D1 == 0 or D2 == 0:
        return {"valid": False, "reason": "Target coincides with a motor pivot; IK undefined."}

    # ── Proximal angles (elbows-out branch) ───────────────────────────────────
    alpha1 = np.arctan2(yp, xp + L0)
    alpha2 = np.arctan2(yp, xp - L0)

    beta1 = np.arccos(np.clip((L1**2 + D1**2 - L2**2) / (2 * L1 * D1), -1.0, 1.0))
    beta2 = np.arccos(np.clip((L1**2 + D2**2 - L2**2) / (2 * L1 * D2), -1.0, 1.0))

    theta1 = alpha1 + beta1   # left motor: elbow bends upward / outward
    theta2 = alpha2 - beta2   # right motor: elbow bends upward / outward

    # ── Elbows-out check ──────────────────────────────────────────────────────
    E1x = -L0 + L1 * np.cos(theta1)
    E2x =  L0 + L1 * np.cos(theta2)

    if not (E1x < xp and E2x > xp):
        return {"valid": False, "reason": "Violates elbows-out constraint."}

    # ── Transmission / comfort angle check ───────────────────────────────────
    cos_gamma_max = np.cos(np.radians(comfort_angle_deg))
    cos_g1 = (L1**2 + L2**2 - D1**2) / (2 * L1 * L2)
    cos_g2 = (L1**2 + L2**2 - D2**2) / (2 * L1 * L2)

    if np.abs(cos_g1) > cos_gamma_max:
        return {"valid": False, "reason": "Left arm near-singular (poor transmission angle)."}
    if np.abs(cos_g2) > cos_gamma_max:
        return {"valid": False, "reason": "Right arm near-singular (poor transmission angle)."}

    # ── Distal link angle (left arm) ─────────────────────────────────────────
    E1       = np.array([-L0 + L1 * np.cos(theta1), L1 * np.sin(theta1)])
    distal1  = np.array([xp - E1[0], yp - E1[1]])
    phi1     = np.arctan2(distal1[1], distal1[0])

    return {
        "valid":      True,
        "x":          xp,
        "y":          yp,
        "theta1":     theta1,
        "theta2":     theta2,
        "theta1_deg": np.degrees(theta1),
        "theta2_deg": np.degrees(theta2),
        "phi1_deg":   np.degrees(phi1),
    }


# ─────────────────────────────────────────────────────────────────────────────
# Forward Kinematics
# ─────────────────────────────────────────────────────────────────────────────

def forward_kinematics(
    theta1: float,
    theta2: float,
    L1: float         = L1,
    L2: float         = L2,
    motor_dist: float = MOTOR_DIST,
) -> dict | None:
    """
    Compute full robot geometry from proximal joint angles.

    The end-effector is the upper intersection of the two distal-link circles
    (positive-Y solution).

    Parameters
    ----------
    theta1, theta2 : proximal joint angles (radians, from +x axis).

    Returns
    -------
    dict with numpy arrays:
        M1, M2 : base pivot positions  (2,)
        E1, E2 : elbow positions       (2,)
        P      : end-effector position (2,)
    Returns None if the geometry is degenerate (elbows too close / too far).
    """
    L0 = motor_dist / 2.0
    M1 = np.array([-L0, 0.0])
    M2 = np.array([ L0, 0.0])

    E1 = M1 + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
    E2 = M2 + np.array([L1 * np.cos(theta2), L1 * np.sin(theta2)])

    d = np.linalg.norm(E2 - E1)
    if d > 2 * L2 or d < 1e-9:
        return None

    a    = d / 2.0
    h_sq = L2**2 - a**2

    if h_sq < 0:
        if h_sq > -1e-8:   # tolerate tiny floating-point underrun
            h_sq = 0.0
        else:
            return None

    h    = np.sqrt(h_sq)
    Pmid = E1 + a * (E2 - E1) / d
    perp = np.array([-(E2[1] - E1[1]), E2[0] - E1[0]]) / d

    P_up   = Pmid + h * perp
    P_down = Pmid - h * perp
    P      = P_up if P_up[1] >= P_down[1] else P_down

    return {"M1": M1, "M2": M2, "E1": E1, "E2": E2, "P": P}


# ─────────────────────────────────────────────────────────────────────────────
# Workspace mask  (for visualisation / reachability checks)
# ─────────────────────────────────────────────────────────────────────────────

def calculate_workspace(
    L1: float               = L1,
    L2: float               = L2,
    motor_dist: float       = MOTOR_DIST,
    comfort_angle_deg: float = COMFORT_ANGLE_DEG,
    x_range: tuple          = (-300, 300),
    y_range: tuple          = (0, 450),
    resolution: int         = 400,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build a boolean reachability mask over a 2-D grid.

    Applies the same elbows-out and comfort-angle filters as solve_ik.

    Returns
    -------
    mask : (ny, nx) bool array  — True where the point is reachable.
    X    : (ny, nx) float array — meshgrid x-coordinates.
    Y    : (ny, nx) float array — meshgrid y-coordinates.
    """
    L0 = motor_dist / 2.0
    ny = int(resolution * (y_range[1] - y_range[0]) / (x_range[1] - x_range[0]))

    x = np.linspace(x_range[0], x_range[1], resolution)
    y = np.linspace(y_range[0], y_range[1], ny)
    X, Y = np.meshgrid(x, y)

    D1 = np.sqrt((X + L0)**2 + Y**2)
    D2 = np.sqrt((X - L0)**2 + Y**2)

    reach1 = (D1 <= L1 + L2) & (D1 >= abs(L1 - L2))
    reach2 = (D2 <= L1 + L2) & (D2 >= abs(L1 - L2))

    with np.errstate(invalid='ignore', divide='ignore'):
        alpha1 = np.arctan2(Y, X + L0)
        alpha2 = np.arctan2(Y, X - L0)

        beta1 = np.arccos(np.clip((L1**2 + D1**2 - L2**2) / (2 * L1 * D1), -1.0, 1.0))
        beta2 = np.arccos(np.clip((L1**2 + D2**2 - L2**2) / (2 * L1 * D2), -1.0, 1.0))

        theta1 = alpha1 + beta1
        theta2 = alpha2 - beta2

        E1x = -L0 + L1 * np.cos(theta1)
        E2x =  L0 + L1 * np.cos(theta2)
        elbows_out = (E1x < X) & (E2x > X)

        cos_gamma_max = np.cos(np.radians(comfort_angle_deg))
        cos_g1 = (L1**2 + L2**2 - D1**2) / (2 * L1 * L2)
        cos_g2 = (L1**2 + L2**2 - D2**2) / (2 * L1 * L2)
        comfort = (np.abs(cos_g1) <= cos_gamma_max) & (np.abs(cos_g2) <= cos_gamma_max)

    mask = reach1 & reach2 & elbows_out & comfort
    return mask, X, Y


# ─────────────────────────────────────────────────────────────────────────────
# Motor command helper
# ─────────────────────────────────────────────────────────────────────────────

def angles_to_commands(
    theta1_deg: float,
    theta2_deg: float,
    home_angle: float = HOME_ANGLE,
    m1_sign: int      = M1_SIGN,
    m2_sign: int      = M2_SIGN,
) -> tuple[float, float]:
    """
    Convert IK proximal angles to the signed delta values expected by the
    ESP32 firmware (degrees relative to the physical home position).

    Parameters
    ----------
    theta1_deg, theta2_deg : angles from solve_ik (degrees from +x axis).
    home_angle             : IK angle that corresponds to encoder zero.
    m1_sign, m2_sign       : ±1 to correct for physical winding direction.

    Returns
    -------
    (cmd1, cmd2) : float pair ready to format as "cmd1,cmd2\\n" for serial.
    """
    cmd1 = m1_sign * (theta1_deg - home_angle)
    cmd2 = m2_sign * (theta2_deg - home_angle)
    return cmd1, cmd2


# ─────────────────────────────────────────────────────────────────────────────
# Quick self-test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_points = [
        (0.0,   390.0),   # centre, near far limit
        (0.0,   200.0),   # centre, mid range
        (80.0,  250.0),   # right of centre
        (-80.0, 250.0),   # left of centre
        (500.0, 300.0),   # out of reach — should fail
    ]

    print(f"{'Point':>20}  {'Valid':>5}  {'θ1 (°)':>10}  {'θ2 (°)':>10}  {'φ1 (°)':>10}  {'cmd1':>8}  {'cmd2':>8}")
    print("-" * 82)
    for xp, yp in test_points:
        r = solve_ik(xp, yp)
        if r["valid"]:
            c1, c2 = angles_to_commands(r["theta1_deg"], r["theta2_deg"])
            fk = forward_kinematics(r["theta1"], r["theta2"])
            ee = fk["P"] if fk else (float("nan"), float("nan"))
            print(f"  ({xp:6.1f}, {yp:6.1f})  {str(r['valid']):>5}  "
                  f"{r['theta1_deg']:>10.3f}  {r['theta2_deg']:>10.3f}  "
                  f"{r['phi1_deg']:>10.3f}  "
                  f"{c1:>8.3f}  {c2:>8.3f}   "
                  f"FK=({ee[0]:.1f}, {ee[1]:.1f})")
        else:
            print(f"  ({xp:6.1f}, {yp:6.1f})  {str(r['valid']):>5}  {r['reason']}")