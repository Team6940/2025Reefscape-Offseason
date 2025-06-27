package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputStream.GetField;
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
import frc.robot.Constants.FieldConstants;
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
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.ClimbCommands.SemiAutoClimbCommand;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
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
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive 
            
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        /* DEFAULT COMMANDS */  // TODO
        chassis.registerTelemetry(logger::telemeterize);

        chassis.setDefaultCommand(
            // chassis will execute this command periodically
            chassis.applyRequest(() -> drive
                    .withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed / 1.)    //TODO: change speed here
                    .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed / 1.)  //TODO: change speed here
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate / 1.)//TODO: change speed here
            ));

        /**
         * Driver Controller:
         * Left Stick: Translation
         * Right Stick: Rotation
         * 
         * Left bumper: Hybrid Intake (deploy the ground intake and stop after releasing the bumper)
         * Left trigger: Algae Intake   ///
         * Right Bumper: Hybrid Scoring   ///
         * Right Trigger: Algae Scoring   ///
         * 
         * Left Stick Pressed: Extend Climber (toggle command) & Climb Release (press again)   ///
         * Right Stick Pressed: Retract Climber   ///
         * 
         * X: Reset Gyro   ///
         * Y: System Initialize (Reset All Subsystems)   ///TODO
         * A: (Auto) Confirm Coral Scoring Position Selection   
         * B: Drive To Net Scoring Point   ///(written in command)
         *  
         * povDown: (Auto) ReefLevelIndex - 1  ///
         * povUP: (Auto) ReefLevelIndex + 1    ///
         * povLeft: (Auto) ReefPoseIndex - 1    ///
         * povRight: (Auto) ReefPoseIndex + 1   ///
         */

        /* Operator Controller: 
         * povUp: Teleop Select 'Upper' Face of the Reef
         * povDown: Teleop Select 'Lower' Face of the Reef
         * povLeft: Teleop Select Left Hand Side Face
         * povRight: Teleop Select Right Hand Side Face
         * 
         * **   The 'FaceIndex' refers to the different 'sides' of the hexogonal reef,
         * **   which are numbered 0-5, going clockwise,
         * **   e.g, ReefPoseIndex 1,2 -> FaceIndex 0
         * **                      3,4 -> FaceIndex 1
         * **                      5,6 -> FaceIndex 2
         * **                      7,8 -> FaceIndex 3
         * **                     9,10 -> FaceIndex 4
         * **                    11,12 -> FaceIndex 5     TODO needed here, decide whether it's 0-5 or 1-6
         * 
         * LB: Select Left          
         * RB: Select Right
         * 
         * ** The left and right is decided by the following method,
         * ** imaging facing the 'side' which you are trying to score the coral at,
         * ** the left is the side which is on your left hand side, and the right is the side which is on your right hand side.
         * 
         * A: Teleop Select L1
         * B: Teleop Select L2
         * X: Teleop Select L3
         * Y: Teleop Select L4
        */


        /* ---------------------------------------- DRIVER CONTROLLER ----------------------------------------*/
        /* Sticks */  
        // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
        driverController.leftStick().toggleOnTrue(new SemiAutoClimbCommand(Button.kLeftStick,Button.kRightStick)); // TODO change configure bindings

        /* Bumpers & Triggers */
        driverController.rightBumper().whileTrue(superStructure.runOnce(() -> superStructure.getHybridCoralCommand(Button.kRightBumper)));
        driverController.rightTrigger().whileTrue(superStructure.runOnce(() -> superStructure.getHybridAlgaeCommand(Button.kRightBumper)));
        driverController.leftBumper().toggleOnTrue(new ToggleIntake(grArm, intaker));
        driverController.leftBumper().toggleOnFalse(superStructure.runOnce(() -> superStructure.getHybridCoralIntakeCommand()));
        driverController.leftTrigger().whileTrue(superStructure.runOnce(() -> superStructure.getHybridAlgaeIntakeCommand(Button.kLeftTrigger)));

        /* Buttons */
        driverController.x().onTrue(new InstantCommand(() -> chassis.resetPose(new Pose2d(0, 4, new Rotation2d()))));
        // driverController.x().onTrue(chassis.runOnce(() -> chassis.seedFieldCentric())); // TODO seed field-centric heading
        driverController.a().whileTrue(RobotContainer.chassis.followPPPath("1"));
        driverController.y().whileTrue(RobotContainer.chassis.followPPPath("2"));
        driverController.b().whileTrue(chassis.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        /* Povs */
        operatorController.povUp().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(1)));
        operatorController.povDown().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(-1)));
        operatorController.povLeft().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefPoseIndex(1)));
        operatorController.povRight().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefPoseIndex(-1)));

        ///THOSE ARE FOR TESTING.
        driverController.leftTrigger().whileTrue(new ToggleIntake(grArm, intaker)); 
        
        driverController.povRight().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[1]));
        driverController.povUp().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[2]));
        driverController.povLeft().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[3]));
        driverController.povDown().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.elevatorHeights[4]));


        /* ---------------------------------------- OPERATOR CONTROLLER ----------------------------------------*/

        /* Sticks */
        /* Bumpers & Triggers */
        /* Buttons */
        /* Povs */
        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
