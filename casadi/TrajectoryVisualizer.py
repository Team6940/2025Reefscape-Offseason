import pandas as pd
import sys
import matplotlib.pyplot as plt

# Load trajectory CSV
if len(sys.argv) < 2:
    print("Usage: python plot_traj.py <path_to_csv>")
    sys.exit(1)

file_path = sys.argv[1]
df = pd.read_csv(file_path, skiprows = 1)

# Create a figure with 3 subplots (stacked vertically)
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# -----------------------------
# 1. Elevator height
# -----------------------------
axs[0].plot(df.time, df.height, color='blue', label='Height [m]')
axs[0].set_ylabel('Height [m]')
axs[0].legend()
axs[0].grid(True)

# -----------------------------
# 2. Arm angle
# -----------------------------
axs[1].plot(df.time, df.angle_deg, color='green', label='Angle [deg]')
axs[1].set_ylabel('Angle [deg]')
axs[1].legend()
axs[1].grid(True)

# -----------------------------
# 3. Velocities
# -----------------------------
axs[2].plot(df.time, df.dh, color='red', label='dh [m/s]')
axs[2].plot(df.time, df.dtheta_rad_s, color='orange', label='dtheta [rad/s]')
axs[2].set_xlabel('Time [s]')
axs[2].set_ylabel('Velocity')
axs[2].legend()
axs[2].grid(True)

# Adjust spacing
plt.tight_layout()
plt.show()
