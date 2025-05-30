package frc.robot;

import static edu.wpi.first.units.Units.*;

// import javax.print.StreamPrintService;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.*;

public class RobotContainer {

    public static final String m_Limelight = "limelight-front";
    
    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(0); 
      public static final XboxController traditionalDriverController = new XboxController(0);

    public static final CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
        

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // Originally 3/4 of a rotation per second max angular velocity

    //TODO: change deadband here
    public static final double driveDeadband = 0.04;
    public static final double rotateDeadband = 0.04;    
             
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * driveDeadband).withRotationalDeadband(MaxAngularRate * rotateDeadband) 
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        /* DEFAULT COMMANDS */
        chassis.registerTelemetry(logger::telemeterize); //TODO: copied from 2025Reefscape


        /* DRIVER CONTROLLER */

        /**  
        * The ideal control logic is such: 
        * Left Stick: Translation
        * Right Stick: Rotation
        * Left bumper: Hybrid Intake
        * Left trigger: Outtake from the back
        * Right Bumper + Right Trigger: Hybrid Scoring
        * Left Stick Pressed: Extend Climber and Intake
        * Right Stick Pressed: Retract Climber
        * X: Algae Romoval After Scoring
        * Y: Increase Reef Level Index
        * A: Decrease Reef Level Index
        * B: Outtake from the front
        * povDown: switch hybrid intake and elevator control
        * povUP: resetFieldCentric
        * povLeft: 
        * povRight: 
   */

        /* Povs */ //TODO
        driverController.povUp().onTrue(chassis.runOnce(() -> chassis.seedFieldCentric())); // Bind the left trigger to seed the field-centric heading.


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        chassis.setDefaultCommand(
            // chassis will execute this command periodically
            chassis.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed /1.) // Drive forward with negative Y (forward) //TODO: change speed here
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed/ 1.) // Drive left with negative X (left) //TODO: change speed here
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate / 1.) // Drive counterclockwise with negative X (left) //TODO: change speed here
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            chassis.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(chassis.applyRequest(() -> brake));
        driverController.b().whileTrue(chassis.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        chassis.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
