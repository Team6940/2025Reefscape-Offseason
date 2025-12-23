import casadi as ca  
import numpy as np

FILE_NAME = "Trajectory_L4ToStow.csv"

# ============================================================
# 1. Time parameters
# ============================================================
T = 0.7 # total time (s), try reduce this to speed up motion
        # make sure this number is n*0.02, n in N*    
dt = 0.02               
N = int(T / dt)         

# ============================================================
# 2. Boundary conditions
# ============================================================
h0, hf = 0.7, 0.0       
theta0 = np.deg2rad(-235.0)
thetaf = np.deg2rad(-93.0)
dh0, dhf = 0.0, 0.0
dtheta0, dthetaf = 0.0, 0.0

# ============================================================
# 3. Symbolic states and controls
# ============================================================
h      = ca.MX.sym("h")
theta  = ca.MX.sym("theta")
dh     = ca.MX.sym("dh")
dtheta = ca.MX.sym("dtheta")
x = ca.vertcat(h, theta, dh, dtheta)

ah     = ca.MX.sym("ah")
atheta = ca.MX.sym("atheta")
u = ca.vertcat(ah, atheta)

# ============================================================
# 4. Continuous dynamics
# ============================================================
xdot = ca.vertcat(dh, dtheta, ah, atheta)
f = ca.Function("f", [x, u], [xdot])

# ============================================================
# 5. Decision variables
# ============================================================
X = ca.MX.sym("X", 4, N + 1)
U = ca.MX.sym("U", 2, N)

# ============================================================
# 6. Dynamics constraints
# ============================================================
g = []
for k in range(N):
    x_next = X[:, k] + dt * f(X[:, k], U[:, k])
    g.append(X[:, k + 1] - x_next)

# ============================================================
# 7. Boundary constraints
# ============================================================
g += [
    X[0, 0] - h0, X[1, 0] - theta0, X[2, 0] - dh0, X[3, 0] - dtheta0,
    X[0, -1] - hf, X[1, -1] - thetaf, X[2, -1] - dhf, X[3, -1] - dthetaf,
]

# ============================================================
# 8. Cost function (smooth + soft arm motion constraint)
# ============================================================
J = 0
h_min_for_arm = 0.1  # minimum height for arm to move

for k in range(N):
    # Smooth motion: penalize accelerations
    J += 1.0 * ca.sumsqr(U[:, k])
    J += 0.1 * ca.sumsqr(X[2:4, k])  # penalize velocities (reduces flicker)

    # Penalize arm motion if elevator below threshold
    # Smooth penalty using max(0, h_min - h)
    arm_penalty = ca.fmax(0, h_min_for_arm - X[0, k])
    penalty_strength = 1e6 # penalty weight
    J += penalty_strength * arm_penalty**2 * X[3, k]**2

# ============================================================
# 9. NLP formulation
# ============================================================
vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
g = ca.vertcat(*g)
nlp = {"x": vars, "f": J, "g": g}
solver = ca.nlpsol("solver", "ipopt", nlp)

# ============================================================
# 10. Solve
# ============================================================
x0 = np.zeros(vars.shape[0])
sol = solver(x0=x0, lbg=0, ubg=0)

# ============================================================
# 11. Extract solution
# ============================================================
sol_array = np.array(sol["x"]).flatten()
NX = 4*(N+1)
X_sol = sol_array[:NX].reshape((4, N+1), order="F")
U_sol = sol_array[NX:].reshape((2, N), order="F")

# ============================================================
# 12. Time vector
# ============================================================
time = np.linspace(0, T, N+1)

# ============================================================
# 13. Export trajectory
# ============================================================
angle_deg = np.rad2deg(X_sol[1])
traj = np.vstack([time, X_sol[0], angle_deg, X_sol[2], X_sol[3]]).T
np.savetxt(FILE_NAME, traj, delimiter=",",
           header="time,height,angle_deg,dh,dtheta_rad_s", comments="")

# ============================================================
# 14. Sanity check
# ============================================================
print("Initial state:", X_sol[:,0])
print("Final state:", X_sol[:,-1])
print("Trajectory saved to", FILE_NAME)
