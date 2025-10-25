package frc.robot.containers;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Telemetry;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.SimConstants.FieldSimConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Controller.KeyboardController;

public class SimContainer implements BaseContainer {

    /* INITIALIZE */

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public static final String m_Limelight = "limelight";

    public static final SuperStructure superStructure = SuperStructure.getInstance();
    public static final CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static final ArmSubsystem arm = ArmSubsystem.getInstance();
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public static final GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    public static final IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    public static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // TODO: change deadband here 

    public static final double driveDeadbandKeyboard = 0.05;
    public static final double rotateDeadbandKeyboard = 0.09;

    public static final double triggerThresholdSim = 1.; //TODO:change base on uesd controller

    public static final KeyboardController keyboardController = new KeyboardController();
    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(
            0,
            triggerThresholdSim);
    public static final ImprovedCommandXboxController operatorController = new ImprovedCommandXboxController(
            1,
            triggerThresholdSim);
    public static final XboxController traditionalDriverController = new XboxController(0);
    public int getPOV() { return traditionalDriverController.getPOV();}

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * driveDeadbandKeyboard).withRotationalDeadband(MaxAngularRate * rotateDeadbandKeyboard)
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive                        
                                     
    public SimContainer() {configureBindings();}

    private void configureBindings() {

        chassis.registerTelemetry(logger::telemeterize);

        /* REMEMBER TO CHANGE TO (0,0,Rotation2d()) WHEN CALIBRATING COMPONENTS. */
        chassis.resetPose(
                new Pose2d(
                        FieldSimConstants.CALIBRATION_X,
                        FieldSimConstants.CALIBRATION_Y,
                    new Rotation2d())); 

        chassis.setDefaultCommand(
                chassis.applyRequest(() -> drive
                        .withVelocityX(
                            -driverController.getLeftY() 
                                * Math.abs(driverController.getLeftY()) * MaxSpeed * 0.9)
                        .withVelocityY(
                            -driverController.getLeftX() 
                                * Math.abs(driverController.getLeftX()) * MaxSpeed * 0.9)
                        .withRotationalRate(
                                    -driverController.getRightX() * MaxAngularRate * 1.6)
                        ));

        driverController.rightBumper().onTrue(new NewCoralAlignSequence(Button.kRightStick));
        driverController.rightBumper().whileTrue(new ToggleIntake(grArm, intaker));
        
            keyboardController.up().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    3.)
                            .withVelocityY(
                                    0.)
                            .withRotationalRate(
                                    0.))
                                    );
            keyboardController.down().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    -3.)
                            .withVelocityY(
                                    0.)
                            .withRotationalRate(
                                    0.))
                                    );
            keyboardController.left().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    0.)
                            .withVelocityY(
                                    3.)
                            .withRotationalRate(
                                    0.)));
            keyboardController.right().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    0.)
                            .withVelocityY(
                                    -3.)
                            .withRotationalRate(
                                    0.)));
            keyboardController.z().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    0.)
                            .withVelocityY(
                                    0.)
                            .withRotationalRate(
                                    Math.toRadians(-360.))));
            keyboardController.x().toggleOnTrue(
                    chassis.applyRequest(() -> drive
                            .withVelocityX(
                                    0.)
                            .withVelocityY(
                                    0.)
                            .withRotationalRate(
                                    Math.toRadians(360.))));
        
}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
