package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.robot.util.Simulation.MyDriveTrainSimulation;

public class IntakerIOSim implements IntakerIO {
    private IntakeSimulation intakeSimulation;
    private DriveTrainSimulationConfig config = new DriveTrainSimulationConfig(
        Pounds.of(115), // robot weight
        Inches.of(30),  // bumper length
        Inches.of(30),  // bumper width
        Inches.of(30),  //TODO drivetrain track width x
        Inches.of(30),  //TODO drivetrain track width y
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
                        6.75,                                          // driveGearRatio
                        12.8,                                          // steerGearRatio
                        Units.Volts.of(0.2),                 // driveFrictionVoltage
                        Units.Volts.of(0.2),                 // steerFrictionVoltage
                        Units.Meters.of(0.0508),             // wheelRadius
                        Units.KilogramSquareMeters.of(0.01), // momentOfInertia
                        1.                                             // steerInertia
                    )
                );
            }        
        }
    );
    AbstractDriveTrainSimulation driveTrainSimulation = new MyDriveTrainSimulation(config, new Pose2d());
    private double intakeVoltage;
    private double intakeRPS;

    public IntakerIOSim() {
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            driveTrainSimulation,
            // Width of the intake
            Meters.of(0.7), //TODO
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2), //TODO
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 coral
        1
        );
    }
        
    // @Override // Defined by IntakeIO
    // public void setVoltage(double voltage) {
    //     this.intakeVoltage = voltage;
    // }
    
    // @Override // Defined by IntakeIO
    // public void setRPS(double rps) {
    //     this.intakeRPS = rps;
    // }

    // @Override
    // public void updateInputs(IntakerIOInputs inputs) {
    //     intakeSimulation.startIntake();

    //     inputs.motorConnected = true; // Assume always connected in simulation
    //     inputs.motorVoltageVolts = intakeVoltage;
    //     inputs.motorCurrentAmps = 3.; //TODO
    //     inputs.intakerVelocityRPS = intakeRPS;
    // }
}

