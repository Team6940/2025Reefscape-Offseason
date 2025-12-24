import casadi as ca
import numpy as np
from dataclasses import dataclass, asdict

@dataclass
class TrajConstants:
    FILE_NAME: str
    T: float
    dt: float
    h0: float
    hf: float
    theta0_deg: float
    thetaf_deg: float
    dh0: float = 0.0
    dhf: float = 0.0
    dtheta0: float = 0.0
    dthetaf: float = 0.0
    h_min_for_arm: float = 0.3
    penalty_strength: float = 1e7

def generate_trajectory(constants: TrajConstants):
    theta0 = np.deg2rad(constants.theta0_deg)
    thetaf = np.deg2rad(constants.thetaf_deg)
    N = int(constants.T / constants.dt)

    # CasADi symbols
    h = ca.MX.sym("h")
    theta = ca.MX.sym("theta")
    dh = ca.MX.sym("dh")
    dtheta = ca.MX.sym("dtheta")
    x = ca.vertcat(h, theta, dh, dtheta)

    ah = ca.MX.sym("ah")
    atheta = ca.MX.sym("atheta")
    u = ca.vertcat(ah, atheta)

    xdot = ca.vertcat(dh, dtheta, ah, atheta)
    f = ca.Function("f", [x, u], [xdot])

    X = ca.MX.sym("X", 4, N+1)
    U = ca.MX.sym("U", 2, N)

    # Dynamics constraints
    g = []
    for k in range(N):
        x_next = X[:,k] + constants.dt * f(X[:,k], U[:,k])
        g.append(X[:,k+1] - x_next)

    # Boundary constraints
    g += [
        X[0,0]-constants.h0, X[1,0]-theta0, X[2,0]-constants.dh0, X[3,0]-constants.dtheta0,
        X[0,-1]-constants.hf, X[1,-1]-thetaf, X[2,-1]-constants.dhf, X[3,-1]-constants.dthetaf
    ]

    # Cost function
    J = 0
    for k in range(N):
        J += ca.sumsqr(U[:,k])
        J += 0.1 * ca.sumsqr(X[2:4,k])
        arm_penalty = ca.fmax(0, constants.h_min_for_arm - X[0,k])
        J += constants.penalty_strength * arm_penalty**2 * X[3,k]**2

    # NLP solver
    vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
    g = ca.vertcat(*g)
    nlp = {"x": vars, "f": J, "g": g}
    solver = ca.nlpsol("solver", "ipopt", nlp)
    x0 = np.zeros(vars.shape[0])
    sol = solver(x0=x0, lbg=0, ubg=0)

    sol_array = np.array(sol["x"]).flatten()
    NX = 4*(N+1)
    X_sol = sol_array[:NX].reshape((4,N+1), order="F")
    U_sol = sol_array[NX:].reshape((2,N), order="F")

    time = np.linspace(0, constants.T, N+1)
    angle_deg = np.rad2deg(X_sol[1])
    traj = np.vstack([time, X_sol[0], angle_deg, X_sol[2], X_sol[3]]).T

    # Export CSV with constants as comments
    constants_line = ",".join(f"{k}={v}" for k,v in asdict(constants).items())
    with open(constants.FILE_NAME, "w") as f:
        f.write(constants_line + "\n")  # first line
        f.write("time,height,angle_deg,dh,dtheta_rad_s\n")  # CSV header
        np.savetxt(f, traj, delimiter=",")

    print(f"Trajectory saved to {constants.FILE_NAME}")
    return traj

# -------------------------------
# Generate multiple trajectories
# -------------------------------
def generate_multiple_trajectories(constants_list):
    results = {}
    for constants in constants_list:
        traj = generate_trajectory(constants)
        results[constants.FILE_NAME] = traj
    return results

# -------------------------------
# Main execution
# -------------------------------
if __name__ == "__main__":
    traj1 = TrajConstants(
        FILE_NAME="Trajectory_L4ToStow.csv",
        T=0.28, dt=0.001, h0=0.7, hf=0.0,
        theta0_deg=-235.0, thetaf_deg=-93.0,
        h_min_for_arm=0.22, penalty_strength=1e7
    )
    traj2 = TrajConstants(
        FILE_NAME="Trajectory_StowToL4.csv",
        T=0.28, dt=0.001, h0=0.0, hf=0.7,
        theta0_deg=-93.0, thetaf_deg=-235.0,
        h_min_for_arm=0.2, penalty_strength=1e6
    )

    all_trajs = generate_multiple_trajectories([traj1, traj2])
