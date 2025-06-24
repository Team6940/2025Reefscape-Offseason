package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;

// import javax.print.StreamPrintService;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.*;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Chassis.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.commands.ToggleElevatorTest;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class RobotContainer {

    /* INITIALIZE */

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public static final String m_Limelight = "limelight-front";

    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController operatorController = new ImprovedCommandXboxController(1);
    public static final XboxController traditionalDriverController = new XboxController(0);

    public static final SuperStructure superStructure = SuperStructure.getInstance();
    public static final CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static final ArmSubsystem arm = ArmSubsystem.getInstance();
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    public static final IntakerSubsystem intaker = IntakerSubsystem.getInstance();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // TODO: change deadband here
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

        /* DEFAULT COMMANDS */  // TODO
        chassis.registerTelemetry(logger::telemeterize);

        /**
         * Driver Controller:
         * Left Stick: Translation
         * Right Stick: Rotation
         * 
         * Left bumper: Hybrid Intake (deploy the ground intake and stop after releasing the bumper)
         * Left trigger: Algae Intake
         * Right Bumper: Hybrid Scoring
         * Right Trigger: Algae Scoring
         * 
         * Left Stick Pressed: Extend Climber & Climb Action
         * Right Stick Pressed: Retract Climber
         * 
         * X: Reset Gyro
         * Y: System Initialize (Reset All Subsystems)
         * A: (Auto) Confirm Coral Scoring Position Selection
         * B: Drive To Net Scoring Point
         *  
         * povDown: (Auto) ReefLevelIndex - 1
         * povUP: (Auto) ReefLevelIndex + 1
         * povLeft: (Auto) ReefPoseIndex + 1    
         * povRight: (Auto) ReefPoseIndex - 1
         */

        /* Operator Controller: 
         * povUp: Teleop Select L2
         * povDown: Teleop Select L1
         * povLeft: Teleop ReefSideIndex - 1
         * povRight: Teleop ReefSideIndex + 1
         * 
         * **   The 'SideIndex' refers to the different 'sides' of the hexogonal reef,
         * **   which are numbered 0-5, going clockwise,
         * **   e.g, ReefPoseIndex 1,2 -> SideIndex 0
         * **                      3,4 -> SideIndex 1
         * **                      5,6 -> SideIndex 2
         * **                      7,8 -> SideIndex 3
         * **                     9,10 -> SideIndex 4
         * **                    11,12 -> SideIndex 5     TODO needed here, decide whether it's 0-5 or 1-6
         * 
         * X: Select Left            
         * B: Select Right
         * 
         * ** The left and right is decided by the following method,
         * ** imaging facing the 'side' which you are trying to score the coral at,
         * ** the left is the side which is on your left hand side, and the right is the side which is on your right hand side.
         * 
         * A: Teleop Select L3
         * Y: Teleop Select L4
         * 
        */

        /* ---------------------------------------- DRIVER CONTROLLER ----------------------------------------*/
        /* Stick */  
        // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
        chassis.setDefaultCommand(
                // chassis will execute this command periodically
                chassis.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed / 1.)    //TODO: change speed here
                        .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed / 1.)  //TODO: change speed here
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate / 1.)//TODO: change speed here
                ));

        /* Bumpers & Triggers */ // TODO
        driverController.rightBumper().whileTrue(superStructure.runOnce(() -> superStructure.getHybridCoralCommand(Button.kB, Button.kY, Button.kRightBumper))); //TODO have to change logic of this command
        driverController.rightTrigger().whileTrue(superStructure.runOnce(() -> superStructure.getHybridAlgaeCommand(Button.kY,Button.kRightBumper))); //TODO have to change logic of this command


        // driverController.leftBumper().onTrue(chassis.runOnce(() -> chassis.seedFieldCentric())); // seed field-centric heading.

        /* Buttons */
        driverController.x().onTrue(new InstantCommand(() -> chassis.resetPose(new Pose2d(0, 4, new Rotation2d()))));
        driverController.a().whileTrue(RobotContainer.chassis.followPPPath("1"));
        driverController.b().whileTrue(chassis.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
        driverController.y().whileTrue(new InstantCommand(() -> elevator.setHeight(Constants.FieldConstants.elevatorHeights[0])));

        /* Povs */
        operatorController.povUp().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(1)));
        operatorController.povDown().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(-1)));

        ///THOSE ARE FOR TESTING.
        driverController.povRight().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[1]));
        driverController.povUp().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[2]));
        driverController.povLeft().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[3]));
        driverController.povDown().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[4]));

        /* ---------------------------------------- OPERATOR CONTROLLER ----------------------------------------*/
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
