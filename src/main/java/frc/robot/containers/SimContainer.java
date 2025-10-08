package frc.robot.containers;

import static edu.wpi.first.units.Units.*;
import java.util.Set;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Telemetry;
import frc. robot. commands.AlgaeCommands.AlgaeManualScoring;
import frc.robot.commands.ClimbCommands.NewClimbCommand;
import frc.robot.commands.CoralCommands.ScoreL1;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.*;
import frc.robot.subsystems.Controller.KeyboardController;

public class SimContainer {

    /* INITIALIZE */

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public static final String m_Limelight = "limelight";

    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController operatorController = new ImprovedCommandXboxController(1);
    public static final XboxController traditionalDriverController = new XboxController(0); 
    public static final KeyboardController keyboardController = new KeyboardController();

    public static final SuperStructure 
    superStructure = SuperStructure.getInstance();
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
    public static final double driveDeadband = 0.045;
    public static final double rotateDeadband = 0.045;
    public class isL1 implements BooleanSupplier
    {
        @Override
        public boolean getAsBoolean()
        {
            return superStructure.getTargetReefLevelIndex()==1;
        }
    }

    public class isNL1 implements BooleanSupplier {
        @Override
        public boolean getAsBoolean() {
            return superStructure.getTargetReefLevelIndex() != 1;
        }
    }
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * driveDeadband).withRotationalDeadband(MaxAngularRate * rotateDeadband)
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive 
                private BooleanSupplier isL1=new isL1();
                private BooleanSupplier isNL1= new isNL1();
                        
                // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
                // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
            
            
                public SimContainer() {
                    configureBindings();
                }
            
                private void configureBindings() {
            
                    chassis.registerTelemetry(logger::telemeterize);

                    chassis.resetPose(
                        new Pose2d(
                            2,
                            2, 
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
                                    -driverController.getRightX() 
                                        * MaxAngularRate * 0.9)
                        ));
            
                    keyboardController.up().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        3.)
                                    .withVelocityY(
                                        0.)
                                    .withRotationalRate(
                                        0.)
                            )
                    );
                    keyboardController.down().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        -3.)
                                    .withVelocityY(
                                        0.)
                                    .withRotationalRate(
                                        0.)
                            )
                    );
                    keyboardController.left().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        0.)
                                    .withVelocityY(
                                          3.)
                                    .withRotationalRate(
                                        0.)
                            )
                    );
                    keyboardController.right().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        0.)
                                    .withVelocityY(
                                        -3.)
                                    .withRotationalRate(
                                        0.)
                            )
                    );
                    keyboardController.z().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        0.)
                                    .withVelocityY(
                                        0.)
                                    .withRotationalRate(
                                        Math.toRadians(15.))
                            )
                    );
                    keyboardController.x().
                        whileTrue(
                            chassis.applyRequest(() -> drive
                                    .withVelocityX(
                                        0.)
                                    .withVelocityY(
                                        0.)
                                    .withRotationalRate(
                                       Math.toRadians(-15.))
                            )
                    );

//SAME CONTROL LOGIC AS THE REAL ROBOT
            
            
                    /* ---------------------------------------- DRIVER CONTROLLER ----------------------------------------*/
                    /* Sticks */  

                    /* Bumpers & Triggers */
                    driverController.leftBumper().and(isNL1).whileTrue(Commands.defer(() -> superStructure.getNewHybridCoralScoreCommand(Button.kLeftTrigger),Set.of(arm, elevator, shooter, chassis)));
                    driverController.leftBumper().and(isL1).whileTrue(new ScoreL1());
                    driverController.rightBumper().onTrue(new NewCoralAlignSequence(Button.kRightStick));
                    driverController.rightBumper().whileTrue(new ToggleIntake(grArm, intaker));
            
                    /* Buttons */
                    driverController.a().onTrue(new AlgaeManualScoring(Button.kY));
                    driverController.b().whileTrue(Commands.defer(()->superStructure.getManualAlgaeIntakeCommand(),Set.of(arm,elevator,shooter)));
                    driverController.x()
                    .onTrue(
                        new InstantCommand(
                            () -> chassis.resetPose(
                                new Pose2d(2,
                                2, 
                                new Rotation2d()))));
                    driverController.back().onTrue(new NewClimbCommand(Button.kStart));

                    /* Povs */
            

                    /* ---------------------------------------- OPERATOR CONTROLLER ----------------------------------------*/
                    /* Sticks */

                    /* Bumpers & Triggers */
                    operatorController.leftBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.LEFT)));
                    operatorController.rightBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.RIGHT)));

                    /* Buttons */
                    operatorController.a().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(1)));
                    operatorController.b().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(2)));
                    operatorController.x().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(3)));
                    operatorController.y().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(4)));    
            

                    /* ----------------------------------------------- TESING -----------------------------------------------*/
                                
                    // W       W       A       RRRRRR    N     N    IIIII    N     N     GGGGGGG
                    // W       W      A A      R     R   N N   N      I      N N   N    G       
                    // W   W   W     A   A     RRRRRR    N  N  N      I      N  N  N   G      GGGG   
                    // W W   W W    AAAAAAA    R    R    N   N N      I      N   N N    G        G   
                    // W       W   A       A   R     R   N     N    IIIII    N     N     GGGGGGG   
                    
                    
                    // REMEMBER TO DISABLE ALL OTHER FUNCTIONS WHEN STARTING TO TEST ONE FUNC.
                    // REMEMBER TO COMMENT OUT ALL CODES FOR TESTING AFTER TEST IS ALL DONE. 

                    //------------------------------------------------------------------------------------------------------------------
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
