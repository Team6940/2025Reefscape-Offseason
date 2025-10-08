package frc.robot.util.Simulation;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;

public class MyDriveTrainSimulation extends AbstractDriveTrainSimulation {
    public MyDriveTrainSimulation(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        super(config, initialPoseOnField);
    }

    @Override
    public void simulationSubTick() {}
}

