package frc.robot.util.Simulation;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

/**This is used for initializing a custom abstractive drivetrain simulation.
 * </p>See {@link frc.robot.subsystems.Intaker.IntakerIOSim}.
 * */ 

public class MyDriveTrainSimulation extends AbstractDriveTrainSimulation {
    public MyDriveTrainSimulation(DriveTrainSimulationConfig config, Pose2d initialPoseOnField) {
        super(config, initialPoseOnField);
    }

    @SuppressWarnings("unchecked")
    public static DriveTrainSimulationConfig _config = new DriveTrainSimulationConfig(
        Pounds.of(115), // robot weight
        Inches.of(30),  // bumper length
        Inches.of(30),  // bumper width
        Inches.of(28),  //TODO drivetrain track width x
        Inches.of(28),  //TODO drivetrain track width y
        new Supplier<GyroSimulation>(){ //Supplier<GyroSimulation> gyroSimulationFactory,
            public GyroSimulation get() {
                return new GyroSimulation(
                    0.,  // AVERAGE_DRIFTING_IN_30_SECS_MOTIONLESS_DEG
                    0.); // VELOCITY_MEASUREMENT_STANDARD_DEVIATION_PERCENT
            }        
        },
        new Supplier<SwerveModuleSimulation>(){ //Supplier<SwerveModuleSimulation> swerveModuleSimulationFactory
            public SwerveModuleSimulation get() {
                return new SwerveModuleSimulation(
                    new SwerveModuleSimulationConfig(
                        DCMotor.getKrakenX60(1),             // driveMotorModel
                        DCMotor.getKrakenX60(1),             // steerMotorModel
                        6.75,                           // driveGearRatio
                        12.8,                           // steerGearRatio
                        Units.Volts.of(0.2),                 // driveFrictionVoltage
                        Units.Volts.of(0.2),                 // steerFrictionVoltage
                        Units.Meters.of(0.0508),             // wheelRadius
                        Units.KilogramSquareMeters.of(0.01), // the rotational inertia of the entire steering mechanism
                        1.2                // the coefficient of friction of the tires, normally around 1.2
                    )
                );
            }        
        }
    );

    @Override
    public void simulationSubTick() {}
}

