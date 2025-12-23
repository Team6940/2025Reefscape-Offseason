TrajectoryGenerator.py generates an optimized trajectory for a system with an arm and an elevator.
Make sure to customize the constants.
    Use: python TrajectoryGenerator.py 
TrajectoryVisualizer.py visualizes the generated trajectory.
    Use: python TrajectoryVisualizer.py <trajectory_path>

important formulas: 
T_min >= max(sqrt(2*delta_h/acc_h_max), sqrt(2*delta_theta/acc_theta_max))