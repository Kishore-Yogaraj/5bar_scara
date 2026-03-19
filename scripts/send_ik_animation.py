import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import serial
import time

# =========================================================
# PARAMETERS
# =========================================================
L1 = 200.0          # Proximal link length (mm)
L2 = 200.0          # Distal link length (mm)
motor_dist = 100.0  # Distance between base motors (mm)

rect_w = 250
rect_h = 170
y_offset = 90

comfort_angle_deg = 15.0

# Serial config
PORT            = 'COM3'
BAUDRATE        = 115200
HOME_ANGLE      = 90.0   # IK angle (deg from +x) at physical home position
MONITOR_SECONDS = 3.0    # How long to print ESP32 output after each command

# Flip to -1 if a motor moves opposite to expected direction
M1_SIGN = -1
M2_SIGN = -1

# Serial connection
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    time.sleep(2)
    SERIAL_ENABLED = True
    print(f"Connected to ESP32 on {PORT}")
except Exception as e:
    ser = None
    SERIAL_ENABLED = False
    print(f"No serial connection: {e}")

# Workspace display grid
x = np.linspace(-300, 300, 400)
y = np.linspace(0, 450, 300)
X, Y = np.meshgrid(x, y)

# Animation settings
FPS = 60
MOVE_TIME = 1.0   # seconds for each move
ANGLE_TOL = 1e-4


# =========================================================
# HELPER FUNCTIONS
# =========================================================
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def shortest_angle_diff(target, current):
    return wrap_to_pi(target - current)


def calculate_workspace(L1, L2, motor_dist):
    """Same workspace logic as your original code."""
    L0 = motor_dist / 2.0

    D1 = np.sqrt((X + L0)**2 + Y**2)
    D2 = np.sqrt((X - L0)**2 + Y**2)

    reach1 = (D1 <= (L1 + L2)) & (D1 >= abs(L1 - L2))
    reach2 = (D2 <= (L1 + L2)) & (D2 >= abs(L1 - L2))
    valid_reach = reach1 & reach2

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
        comfort1 = (np.abs(cos_g1) <= cos_gamma_max)
        comfort2 = (np.abs(cos_g2) <= cos_gamma_max)

        mask = valid_reach & elbows_out & comfort1 & comfort2

    return mask


def solve_ik(xp, yp, L1, L2, motor_dist, comfort_angle_deg=15.0):
    """
    Solves IK using the same elbows-out branch as your workspace tool.
    Returns proximal joint angles theta1, theta2 measured from global +x.
    """
    L0 = motor_dist / 2.0

    left_motor = np.array([-L0, 0.0])
    right_motor = np.array([L0, 0.0])
    target = np.array([xp, yp])

    D1 = np.hypot(xp + L0, yp)
    D2 = np.hypot(xp - L0, yp)

    reach1 = (abs(L1 - L2) <= D1 <= (L1 + L2))
    reach2 = (abs(L1 - L2) <= D2 <= (L1 + L2))
    if not (reach1 and reach2):
        return {"valid": False, "reason": "Outside basic reachability."}

    if D1 == 0 or D2 == 0:
        return {"valid": False, "reason": "Point is at motor origin; IK undefined."}

    alpha1 = np.arctan2(yp, xp + L0)
    alpha2 = np.arctan2(yp, xp - L0)

    beta1 = np.arccos(np.clip((L1**2 + D1**2 - L2**2) / (2 * L1 * D1), -1.0, 1.0))
    beta2 = np.arccos(np.clip((L1**2 + D2**2 - L2**2) / (2 * L1 * D2), -1.0, 1.0))

    # Same branch as your workspace code
    theta1 = alpha1 + beta1
    theta2 = alpha2 - beta2

    E1 = np.array([
        -L0 + L1 * np.cos(theta1),
         0.0 + L1 * np.sin(theta1)
    ])

    E2 = np.array([
         L0 + L1 * np.cos(theta2),
         0.0 + L1 * np.sin(theta2)
    ])

    elbows_out = (E1[0] < xp) and (E2[0] > xp)
    if not elbows_out:
        return {"valid": False, "reason": "Violates elbows-out constraint."}

    cos_gamma_max = np.cos(np.radians(comfort_angle_deg))
    cos_g1 = (L1**2 + L2**2 - D1**2) / (2 * L1 * L2)
    cos_g2 = (L1**2 + L2**2 - D2**2) / (2 * L1 * L2)
    comfort1 = np.abs(cos_g1) <= cos_gamma_max
    comfort2 = np.abs(cos_g2) <= cos_gamma_max

    if not (comfort1 and comfort2):
        return {"valid": False, "reason": "Too close to singular / poor transmission angle region."}

    return {
        "valid": True,
        "x": xp,
        "y": yp,
        "theta1": theta1,
        "theta2": theta2,
        "theta1_deg": np.degrees(theta1),
        "theta2_deg": np.degrees(theta2)
    }


def forward_kinematics(theta1, theta2, L1, L2, motor_dist):
    """
    Build the robot geometry from proximal angles.
    Elbow points come directly from theta1/theta2.
    End-effector is the intersection of the two distal-link circles.
    We choose the upper intersection (positive y).
    """
    L0 = motor_dist / 2.0
    M1 = np.array([-L0, 0.0])
    M2 = np.array([L0, 0.0])

    E1 = M1 + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
    E2 = M2 + np.array([L1 * np.cos(theta2), L1 * np.sin(theta2)])

    d = np.linalg.norm(E2 - E1)
    if d > 2 * L2 or d < 1e-9:
        return None

    a = d / 2.0
    h_sq = L2**2 - a**2
    if h_sq < 0:
        if h_sq > -1e-8:
            h_sq = 0.0
        else:
            return None
    h = np.sqrt(h_sq)

    Pmid = E1 + a * (E2 - E1) / d

    perp = np.array([-(E2[1] - E1[1]), E2[0] - E1[0]]) / d

    P_up = Pmid + h * perp
    P_down = Pmid - h * perp

    # choose upper solution
    target = P_up if P_up[1] >= P_down[1] else P_down

    return {
        "M1": M1,
        "M2": M2,
        "E1": E1,
        "E2": E2,
        "P": target
    }


# =========================================================
# INITIAL STATE
# =========================================================
# Pick a reasonable initial valid point
initial_point = (0.0, 390.0)
ik0 = solve_ik(initial_point[0], initial_point[1], L1, L2, motor_dist, comfort_angle_deg)
if not ik0["valid"]:
    raise RuntimeError("Initial point is invalid. Choose another starting point.")

current_theta1 = ik0["theta1"]
current_theta2 = ik0["theta2"]

target_theta1 = current_theta1
target_theta2 = current_theta2

start_theta1 = current_theta1
start_theta2 = current_theta2

animating = False
anim_frame = 0
monitor_until = 0.0     # timestamp until which serial output is printed
monitoring_active = False
num_frames = max(2, int(FPS * MOVE_TIME))
selected_point = np.array(initial_point)


# =========================================================
# PLOT SETUP
# =========================================================
mask = calculate_workspace(L1, L2, motor_dist)

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
ax.set_title("5-Bar SCARA Animated Click-to-Move IK", fontsize=14, fontweight='bold')
ax.set_xlabel("X-Axis (mm)")
ax.set_ylabel("Y-Axis (mm)")
ax.grid(True, linestyle='--', alpha=0.6)

cmap = plt.cm.Greens.copy()
cmap.set_under('white', alpha=0)
ax.imshow(
    mask,
    extent=(x.min(), x.max(), y.min(), y.max()),
    origin='lower',
    cmap=cmap,
    vmin=0.5,
    alpha=0.5
)

rect = patches.Rectangle(
    (-rect_w / 2, y_offset), rect_w, rect_h,
    linewidth=2, edgecolor='red', facecolor='none', zorder=10
)
ax.add_patch(rect)

L0 = motor_dist / 2.0
ax.plot([-L0], [0], 'ko', markersize=8, zorder=11)
ax.plot([L0], [0], 'ko', markersize=8, zorder=11)

# Robot drawing objects
left_prox_line, = ax.plot([], [], 'b-', lw=3)
left_dist_line, = ax.plot([], [], 'b--', lw=2)
right_prox_line, = ax.plot([], [], 'm-', lw=3)
right_dist_line, = ax.plot([], [], 'm--', lw=2)

elbow1_plot, = ax.plot([], [], 'bo', markersize=7, zorder=12)
elbow2_plot, = ax.plot([], [], 'mo', markersize=7, zorder=12)
ee_plot, = ax.plot([], [], 'ro', markersize=8, zorder=13)
clicked_plot, = ax.plot([], [], 'rx', markersize=10, mew=2, zorder=13)

info_text = ax.text(
    0.02, 0.98, "",
    transform=ax.transAxes,
    fontsize=11,
    verticalalignment='top',
    bbox=dict(boxstyle='round', facecolor='white', alpha=0.85)
)

ax.set_xlim(-300, 300)
ax.set_ylim(-20, 450)


# =========================================================
# DRAW FUNCTION
# =========================================================
def draw_robot(theta1, theta2):
    geom = forward_kinematics(theta1, theta2, L1, L2, motor_dist)
    if geom is None:
        return

    M1, M2, E1, E2, P = geom["M1"], geom["M2"], geom["E1"], geom["E2"], geom["P"]

    left_prox_line.set_data([M1[0], E1[0]], [M1[1], E1[1]])
    left_dist_line.set_data([E1[0], P[0]], [E1[1], P[1]])

    right_prox_line.set_data([M2[0], E2[0]], [M2[1], E2[1]])
    right_dist_line.set_data([E2[0], P[0]], [E2[1], P[1]])

    elbow1_plot.set_data([E1[0]], [E1[1]])
    elbow2_plot.set_data([E2[0]], [E2[1]])
    ee_plot.set_data([P[0]], [P[1]])

    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)

    info_text.set_text(
        f"Target click: ({selected_point[0]:.1f} mm, {selected_point[1]:.1f} mm)\n"
        f"End effector: ({P[0]:.1f} mm, {P[1]:.1f} mm)\n"
        f"Theta1: {theta1_deg:.2f} deg\n"
        f"Theta2: {theta2_deg:.2f} deg"
    )


# =========================================================
# CLICK HANDLER
# =========================================================
def on_click(event):
    global target_theta1, target_theta2
    global start_theta1, start_theta2
    global animating, anim_frame, selected_point

    if event.inaxes != ax or event.xdata is None or event.ydata is None:
        return

    xp = float(event.xdata)
    yp = float(event.ydata)

    result = solve_ik(xp, yp, L1, L2, motor_dist, comfort_angle_deg)

    if not result["valid"]:
        info_text.set_text(
            f"Clicked point: ({xp:.1f} mm, {yp:.1f} mm)\n"
            f"INVALID: {result['reason']}"
        )
        clicked_plot.set_data([xp], [yp])
        fig.canvas.draw_idle()
        return

    selected_point = np.array([xp, yp])

    # Store current as start
    start_theta1 = current_theta1
    start_theta2 = current_theta2

    # Target angles from IK
    target_theta1 = result["theta1"]
    target_theta2 = result["theta2"]

    # Start animation
    anim_frame = 0
    animating = True

    clicked_plot.set_data([xp], [yp])

    print("--------------------------------------------------")
    print(f"Clicked point: ({xp:.2f}, {yp:.2f}) mm")
    print(f"Target theta1: {np.degrees(target_theta1):.2f} deg")
    print(f"Target theta2: {np.degrees(target_theta2):.2f} deg")

    cmd1 = M1_SIGN * (result["theta1_deg"] - HOME_ANGLE)
    cmd2 = M2_SIGN * (result["theta2_deg"] - HOME_ANGLE)
    command = f"{cmd1:.2f},{cmd2:.2f}\n"
    if SERIAL_ENABLED:
        ser.write(command.encode())
        ser.flush()
    print(f"  >> Sent: {command.strip()}")
    print(f"--- Monitoring for {MONITOR_SECONDS:.0f}s ---")
    global monitor_until, monitoring_active
    monitor_until = time.time() + MONITOR_SECONDS
    monitoring_active = True


fig.canvas.mpl_connect('button_press_event', on_click)


def on_key(event):
    if event.key == 'h' and SERIAL_ENABLED:
        ser.write(b"HOME\n")
        ser.flush()
        print("HOME sent — encoders reset to 0 (arms must be at home position).")

fig.canvas.mpl_connect('key_press_event', on_key)


# =========================================================
# ANIMATION LOOP
# =========================================================
def update_animation(frame):
    global current_theta1, current_theta2
    global animating, anim_frame
    global monitoring_active

    if SERIAL_ENABLED and monitoring_active:
        now = time.time()
        if now < monitor_until:
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                print(line)
        else:
            # Drain any remaining bytes and close out the window
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                print(line)
            print("---\n")
            monitoring_active = False

    if animating:
        anim_frame += 1
        s = min(anim_frame / num_frames, 1.0)

        # Smooth interpolation
        s_smooth = 3 * s**2 - 2 * s**3

        d1 = shortest_angle_diff(target_theta1, start_theta1)
        d2 = shortest_angle_diff(target_theta2, start_theta2)

        current_theta1 = start_theta1 + s_smooth * d1
        current_theta2 = start_theta2 + s_smooth * d2

        if s >= 1.0 or (abs(shortest_angle_diff(target_theta1, current_theta1)) < ANGLE_TOL and
                        abs(shortest_angle_diff(target_theta2, current_theta2)) < ANGLE_TOL):
            current_theta1 = target_theta1
            current_theta2 = target_theta2
            animating = False

    draw_robot(current_theta1, current_theta2)

    return (
        left_prox_line, left_dist_line,
        right_prox_line, right_dist_line,
        elbow1_plot, elbow2_plot,
        ee_plot, clicked_plot, info_text
    )


# Initial draw
draw_robot(current_theta1, current_theta2)

ani = FuncAnimation(fig, update_animation, interval=1000 / FPS, blit=False, cache_frame_data=False)

plt.show()