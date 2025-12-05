package frc.robot.containers;

import static edu.wpi.first.units.Units.*;
import java.util.Set;
import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.Telemetry;
import frc. robot. commands.AlgaeCommands.AlgaeManualScoring;
import frc.robot.commands.ClimbCommands.NewClimbCommand;
import frc.robot.commands.CoralCommands.ScoreL1;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.constants.SimConstants.FieldSimConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.*;

public class RobotContainer implements BaseContainer{

    /* INITIALIZE */

    // TODO: change chassis drive deadband here
    public static final double driveDeadband = 0.05;
    public static final double rotateDeadband = 0.09;
    public static final double triggerThresholdReal = 1.; //0.3

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    public static final String m_Limelight = "limelight";

    public static final ImprovedCommandXboxController driverController = new ImprovedCommandXboxController(
        0,
        triggerThresholdReal);
    public static final ImprovedCommandXboxController operatorController = new ImprovedCommandXboxController(
        1,
        triggerThresholdReal);
    public static final XboxController traditionalDriverController = new XboxController(0);

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


    public class isL1 implements BooleanSupplier{
    @Override
        public boolean getAsBoolean(){return superStructure.getTargetReefLevelIndex()==1;}
    }

    public class isNL1 implements BooleanSupplier {
        @Override
        public boolean getAsBoolean() {return superStructure.getTargetReefLevelIndex() != 1;}
    }
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * driveDeadband).withRotationalDeadband(MaxAngularRate * rotateDeadband)
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive 

                private BooleanSupplier isL1=new isL1();
                private BooleanSupplier isNL1= new isNL1();
                        
                // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
                // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
            
    public int getPOV() {
        return traditionalDriverController.getPOV();
    }

    public RobotContainer() {
                chassis.resetPose(
                new Pose2d(
                        FieldSimConstants.CALIBRATION_X,
                        FieldSimConstants.CALIBRATION_Y,
                    new Rotation2d()));
                     configureBindings();
                } 
            
    private void configureBindings() {
            
        chassis.registerTelemetry(logger::telemeterize);
                    // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
                    // chassis will execute this command periodically;

                    chassis.setDefaultCommand(
                        chassis.applyRequest(() -> drive
                                .withVelocityX(
                                    -driverController.getLeftY() 
                                        * Math.abs(driverController.getLeftY()) * MaxSpeed * 0.9)
                                .withVelocityY(
                                    -driverController.getLeftX() 
                                        * Math.abs(driverController.getLeftX()) * MaxSpeed * 0.9)
                                .withRotationalRate(
                                    -driverController.getRightX() * MaxAngularRate * 0.9)
                        ));

                    /** 
                     /////////////////// OPERATION LOGIC FOR DRIVER/OPERATOR CONTROLLER ///////////////////
                     * Driver Controller:
                     * Left Stick: Translation
                     * Right Stick: Rotation
                     * 
                     * LB: Hybrid Intake
                     * LT: L1 Coral Scoring
                     * RB: Coral Align Sequence
                     * RT: Toggle Intake
                     * 
                     * X: Reset Gyro
                     * Y: Zero Elevator
                     * Back: Climb Mode
                     * A: Algae Manual Scoring  
                     * B: Algae Manual Intake
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
            
            
                    /* ---------------------------------------- DRIVER CONTROLLER ----------------------------------------*/
                    /* Sticks */  

                    /* Bumpers & Triggers */
                    driverController.leftBumper()
                        .and(isNL1).whileTrue(Commands.defer(() -> superStructure.getNewHybridCoralScoreCommand(Button.kLeftTrigger),Set.of(arm, elevator, shooter, chassis)))
                        .and(isL1).whileTrue(new ScoreL1());
                    // driverController.leftTrigger().onTrue(Commands.runOnce(()->ScoringSimulator.setCoralScoreSim(SuperStructure.m_targetReefLevelIndex)));//FOR SIM
                    driverController.rightBumper().onTrue(new NewCoralAlignSequence(Button.kRightStick)) //right trigger: intake reverse
                        .whileTrue(new ToggleIntake(grArm, intaker));
            
                    /* Buttons */
                    driverController.a().onTrue(new AlgaeManualScoring(Button.kY));
                    driverController.b().whileTrue(Commands.defer(()->superStructure.getManualAlgaeIntakeCommand(),Set.of(arm,elevator,shooter)));
                    driverController.x()
                    .onTrue(new InstantCommand(() -> chassis.resetPose(
                                new Pose2d(2.,2.,new Rotation2d()))));
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
