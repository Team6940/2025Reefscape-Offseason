import casadi as ca
import numpy as np

FILE_NAME = "ArmElevatorTraj.csv"

# ============================================================
# 1. Time parameters (FIXED, KNOWN)
# ============================================================
T = 0.8                 # total motion time [s]
dt = 0.01               # smaller timestep = smoother
N = int(T / dt)         # MUST be integer

# ============================================================
# 2. Boundary conditions (USE RADIANS)
# ============================================================
h0, hf = 0.0, 0.7       # elevator height [m]

theta0 = np.deg2rad(-90.0)    # rad
thetaf = np.deg2rad(-235.0)   # rad

dh0, dhf = 0.0, 0.0
dtheta0, dthetaf = 0.0, 0.0

# ============================================================
# 3. Symbolic states and controls
# ============================================================
# States
h      = ca.MX.sym("h")
theta = ca.MX.sym("theta")
dh     = ca.MX.sym("dh")
dtheta = ca.MX.sym("dtheta")

x = ca.vertcat(h, theta, dh, dtheta)

# Controls (accelerations)
ah     = ca.MX.sym("ah")
atheta = ca.MX.sym("atheta")

u = ca.vertcat(ah, atheta)

# ============================================================
# 4. Continuous dynamics model
# ============================================================
xdot = ca.vertcat(
    dh,        # h_dot
    dtheta,    # theta_dot
    ah,        # h_ddot
    atheta     # theta_ddot
)

f = ca.Function("f", [x, u], [xdot])

# ============================================================
# 5. Decision variables
# ============================================================
X = ca.MX.sym("X", 4, N + 1)   # states over time
U = ca.MX.sym("U", 2, N)       # controls over time

# ============================================================
# 6. Dynamics constraints (Euler integration)
# ============================================================
g = []

for k in range(N):
    x_next = X[:, k] + dt * f(X[:, k], U[:, k])
    g.append(X[:, k + 1] - x_next)

# ============================================================
# 7. Boundary constraints
# ============================================================
g += [
    X[0, 0] - h0,
    X[1, 0] - theta0,
    X[2, 0] - dh0,
    X[3, 0] - dtheta0,

    X[0, -1] - hf,
    X[1, -1] - thetaf,
    X[2, -1] - dhf,
    X[3, -1] - dthetaf,
]

# ============================================================
# 8. Cost function (smooth + stable)
# ============================================================
J = 0

for k in range(N):
    # Penalize acceleration (smooth motion)
    J += 1.0 * ca.sumsqr(U[:, k])

    # Penalize velocity (removes flicker)
    J += 0.1 * ca.sumsqr(X[2:4, k])

# ============================================================
# 9. NLP formulation
# ============================================================
vars = ca.vertcat(
    ca.reshape(X, -1, 1),
    ca.reshape(U, -1, 1)
)

g = ca.vertcat(*g)

nlp = {
    "x": vars,
    "f": J,
    "g": g
}

solver = ca.nlpsol("solver", "ipopt", nlp)

# ============================================================
# 10. Solve
# ============================================================
x0 = np.zeros(vars.shape[0])

sol = solver(
    x0=x0,
    lbg=0,
    ubg=0
)

# ============================================================
# 11. Extract solution (CRITICAL: order='F')
# ============================================================
sol_array = np.array(sol["x"]).flatten()

NX = 4 * (N + 1)

X_sol = sol_array[:NX].reshape((4, N + 1), order="F")
U_sol = sol_array[NX:].reshape((2, N), order="F")

# ============================================================
# 12. Time vector
# ============================================================
time = np.linspace(0, T, N + 1)

# ============================================================
# 13. Export trajectory (STATE ONLY)
# ============================================================
angle_deg = np.rad2deg(X_sol[1])   # convert rad → deg

traj = np.vstack([
    time,
    X_sol[0],      # height
    angle_deg,     # angle (deg)
    X_sol[2],      # dh
    X_sol[3],      # dtheta
]).T

np.savetxt(
    FILE_NAME,
    traj,
    delimiter=",",
    header="time,height,angle_deg,dh,dtheta_rad_s",
    comments="" 
)

# ============================================================
# 14. Sanity check
# ============================================================
print("Initial state (Radians):", X_sol[:, 0])
print("Final state (Radians):", X_sol[:, -1])
print("Trajectory saved to", FILE_NAME)
