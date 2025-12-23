package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CoralCommands.ScoreL1;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.commands.TestCommands.FollowTrajectoryCommand;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Chassis.TunerConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class RobotContainer {

    /* INITIALIZE */

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public static final String m_Limelight = "limelight";

    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController operatorController = new ImprovedCommandXboxController(1);
    public static final XboxController traditionalDriverController = new XboxController(0);

    public static final SuperStructure 
    superStructure = SuperStructure.getInstance();
    public static final CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    public static final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    public static final ArmSubsystem arm = ArmSubsystem.getInstance();
    public static final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    // public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    public static final IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    public static final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    public static final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    public static final VisionSubsystem vision = VisionSubsystem.getInstance();

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
            
            
                public RobotContainer() {
                    configureBindings();
                }
            
                private void configureBindings() {
            
                    /* DEFAULT COMMANDS */  // TODO
                    chassis.registerTelemetry(logger::telemeterize);
                    // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
                    // chassis will execute this command periodically
                    chassis.setDefaultCommand(chassis.run(()->chassis.driveFieldCentric(driverController,1)));
            
            
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
                     * Left Stick Pressed: Extend Climber (toggle command)
                     * Right Stick Pressed: Retract Climber
                     * 
                     * X: Reset Gyro
                     * Y: System Initialize (Reset All Subsystems)
                     * A: (Auto) Confirm Coral Scoring Position Selection   
                     * B: Drive To Net Scoring Point
                     *  
                     * povDown: (Auto) Select Mid2Algae
                     * povUP: None
                     * povLeft: (Auto) Select Up4Corals
                     * povRight: (Auto) Select Down4Corals
                     */
            
                    /* Operator Controller: 
                     * povUp: Teleop Select 'Upper' Face of the Reef
                     * povDown: Teleop Select 'Lower' Face of the Reef
                     * povLeft: Teleop Select Left Hand Side Face
                     * povRight: Teleop Select Right Hand Side Face
                     * 
                     * **   The 'FaceIndex' refers to the different 'sides' of the hexogonal reef,
                     * **   which are numbered 0-5, going clockwise,
                     * **   e.g, ReefPoseIndex 1,2 -> FaceIndex 1
                     * **                      3,4 -> FaceIndex 2
                     * **                      5,6 -> FaceIndex 3
                     * **                      7,8 -> FaceIndex 4
                     * **                     9,10 -> FaceIndex 5
                     * **                    11,12 -> FaceIndex 6     TODO needed here, decide whether it's 0-5 or 1-6
                     *                                                Decided, FaceIndex should be 1-6
                     * 
                     * LB: Select Left          
                     * RB: Select Right
                     * 
                     * LT: Lower Algae Intake
                     * RT: Higher Algae Intake
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
            
                    // W       W       A       RRRRRR    N     N    IIIII    N     N     GGGGGG
                    // W       W      A A      R     R   NN    N      I      NN    N    G       
                    // W   W   W     A   A     RRRRRR    N N   N      I      N N   N   G     GGG   
                    // W W   W W    AAAAAAA    R   R     N  N  N      I      N  N  N    G       G   
                    // W       W   A       A   R    R    N   N N    IIIII    N   N N     GGGGGG    
                    
                    
                    //PLEASE DISABLE ALL OTHER FUNCTIONS WHEN STARTING TO TEST ONE FUNC.
                    //PLEASE COMMENT OUT ALL CODES FOR TESTING AFTER TEST IS ALL DONE.
            
            
                   
                    operatorController.leftBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.LEFT)));
                    operatorController.rightBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.RIGHT)));

                    operatorController.a().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(1))); //down 1
                    operatorController.b().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(2))); //right 2
                    operatorController.x().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(3))); //left 3
                    operatorController.y().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(4))); //up 4
            



                    driverController.x().onTrue(new InstantCommand(() -> chassis.resetPose(new Pose2d(0, 4, new Rotation2d()))));
                    // driverController.back().onTrue(new NewClimbCommand(Button.kStart));
            

                    driverController.rightBumper().onTrue(new NewCoralAlignSequence(Button.kRightTrigger));
                    driverController.rightBumper().whileTrue(new ToggleIntake(grArm, intaker));

            
        

        
                    driverController.leftBumper().and(isNL1).whileTrue(Commands.defer(() -> superStructure.getCoralModeScoringCommand(Button.kRightTrigger),Set.of(arm, elevator, shooter, chassis)));
                    driverController.leftBumper().and(isL1).whileTrue(new ScoreL1());

                    driverController.povDown().onTrue(superStructure.runOnce(() -> superStructure.changeCoralMode()));

                    // driverController.povLeft().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.LEFT)));
                    // driverController.povRight().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.RIGHT)));
                    driverController.povLeft().onTrue(new FollowTrajectoryCommand("Trajectory_StowToL4"));
                    driverController.povRight().onTrue(new FollowTrajectoryCommand("Trajectory_L4ToStow"));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
