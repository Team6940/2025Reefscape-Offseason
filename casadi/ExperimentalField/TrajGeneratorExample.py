import casadi as ca
import numpy as np

# ============================================================
# 1. Time parameters
# ============================================================
T = 0.8                 # total motion time [s]
dt = 0.01               # smaller timestep = smoother
N = int(T / dt)

# ============================================================
# 2. Boundary conditions (USE RADIANS)
# ============================================================
h0, hf = 0.0, 0.7       # elevator height [m]

theta0 = np.deg2rad(-90.0)    # rad
thetaf = np.deg2rad(-235.0)   # rad

dh0, dhf = 0.0, 0.0
dtheta0, dthetaf = 0.0, 0.0

# ============================================================
# 3. State and control definitions
# x = [h, theta, dh, dtheta]
# u = [ah, atheta]
# ============================================================
x = ca.MX.sym("x", 4)
u = ca.MX.sym("u", 2)

h, theta, dh, dtheta = x[0], x[1], x[2], x[3]
ah, atheta = u[0], u[1]

xdot = ca.vertcat(
    dh,
    dtheta,
    ah,
    atheta
)

f = ca.Function("f", [x, u], [xdot])

# ============================================================
# 4. Decision variables
# ============================================================
X = ca.MX.sym("X", 4, N + 1)
U = ca.MX.sym("U", 2, N)

# ============================================================
# 5. Dynamics constraints (Euler integration)
# ============================================================
g = []
for k in range(N):
    x_next = X[:, k] + dt * f(X[:, k], U[:, k])
    g.append(X[:, k + 1] - x_next)

# ============================================================
# 6. Boundary constraints
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

g = ca.vertcat(*g)

# ============================================================
# 7. Cost function (THIS IS CRITICAL)
# ============================================================
J = 0
for k in range(N):
    # penalize acceleration (smooth motion)
    J += 1e-2 * ca.sumsqr(U[:, k])

    # penalize velocity (reduce oscillations)
    J += 1e-1 * ca.sumsqr(X[2:4, k])

# terminal smoothing
J += 10.0 * ca.sumsqr(X[2:4, -1])

# ============================================================
# 8. Variable vector
# ============================================================
vars = ca.vertcat(
    ca.reshape(X, -1, 1),
    ca.reshape(U, -1, 1)
)

# ============================================================
# 9. Bounds (THIS STOPS FLICKERING)
# ============================================================
lbx = []
ubx = []

# State bounds
for _ in range(N + 1):
    lbx += [0.0, np.deg2rad(-270), -2.0, np.deg2rad(-360)]
    ubx += [0.8, np.deg2rad(0),    2.0, np.deg2rad(360)]

# Control bounds
for _ in range(N):
    lbx += [-3.0, np.deg2rad(-720)]
    ubx += [ 3.0, np.deg2rad(720)]

lbx = np.array(lbx)
ubx = np.array(ubx)

# ============================================================
# 10. Solve
# ============================================================
nlp = {"x": vars, "f": J, "g": g}
solver = ca.nlpsol("solver", "ipopt", nlp)

x0 = np.zeros(vars.shape[0])

sol = solver(
    x0=x0,
    lbx=lbx,
    ubx=ubx,
    lbg=0,
    ubg=0
)

# ============================================================
# 11. Extract solution
# ============================================================
sol_array = np.array(sol["x"]).flatten()

NX = 4 * (N + 1)
X_sol = sol_array[:NX].reshape(4, N + 1)
U_sol = sol_array[NX:].reshape(2, N)

# ============================================================
# 12. Save CSV (convert angle back to degrees)
# ============================================================
time = np.linspace(0, T, N + 1)

traj = np.vstack([
    time,
    X_sol[0],
    np.rad2deg(X_sol[1]),
    X_sol[2],
    np.rad2deg(X_sol[3])
]).T

np.savetxt(
    "CorrectTraj.csv",
    traj,
    delimiter=",",
    header="time,height,angle_deg,dheight,dangle_deg",
    comments=""
)

print("Trajectory saved to CorrectTraj.csv")
