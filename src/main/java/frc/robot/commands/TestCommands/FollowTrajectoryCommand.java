package frc.robot.commands.TestCommands;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Library.team6940.TrajectoryPoint;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class FollowTrajectoryCommand extends Command {
    private final ArmSubsystem arm = ArmSubsystem.getInstance();
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final ArrayList<TrajectoryPoint> trajectory = new ArrayList<>();
    private double startTime;

    public FollowTrajectoryCommand(String csvFile) {
        loadTrajectory(csvFile);
        addRequirements(arm, elevator);
    }

    private void loadTrajectory(String csvFile) {
        trajectory.clear();
        File file = new File(Filesystem.getDeployDirectory(), csvFile);

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            br.readLine(); // skip header
            String line;
            while ((line = br.readLine()) != null) {
                String[] tokens = line.split(",");
                trajectory.add(new TrajectoryPoint(
                        Double.parseDouble(tokens[0]), // time
                        Double.parseDouble(tokens[1]), // height
                        Double.parseDouble(tokens[2]), // angle (deg)
                        Double.parseDouble(tokens[3]), // dh
                        Double.parseDouble(tokens[4]) // dtheta
                ));
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void initialize() {
        // Use FPGA timestamp instead of system clock
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsed = Timer.getFPGATimestamp() - startTime;

        // Find segment for interpolation
        TrajectoryPoint prev = trajectory.get(0);
        TrajectoryPoint next = trajectory.get(trajectory.size() - 1);

        for (int i = 0; i < trajectory.size() - 1; i++) {
            if (elapsed >= trajectory.get(i).time && elapsed <= trajectory.get(i + 1).time) {
                prev = trajectory.get(i);
                next = trajectory.get(i + 1);
                break;
            }
        }

        // Linear interpolation
        double ratio = (elapsed - prev.time) / (next.time - prev.time);
        double height = prev.height + ratio * (next.height - prev.height);
        double angle = prev.angleDeg + ratio * (next.angleDeg - prev.angleDeg);

        elevator.setHeight(height);
        arm.setPosition(angle);
        Logger.recordOutput("TrajectoryFollowerDesiredHeight", height);
        Logger.recordOutput("TrajectoryFollowerDesiredDegree", angle);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= trajectory.get(trajectory.size() - 1).time;
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure final positions
        TrajectoryPoint last = trajectory.get(trajectory.size() - 1);
        elevator.setHeight(last.height);
        arm.setPosition(last.angleDeg);
    }
}
