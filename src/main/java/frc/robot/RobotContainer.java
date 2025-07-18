package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.ObjectInputStream.GetField;
import java.util.Set;
import java.util.function.BooleanSupplier;

// import javax.print.StreamPrintService;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.*;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.RobotStatus;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Chassis.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.commands.SetStateIdleDown;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.AlgaeCommands.AlgaeManualIntake;
import frc. robot. commands.AlgaeCommands.AlgaeManualScoring;
import frc.robot.commands.ClimbCommands.NewClimbCommand;
import frc.robot.commands.ClimbCommands.SemiAutoClimbCommand;
import frc.robot.commands.CoralCommands.ScoreL1;
import frc.robot.commands.GroundIntakeCommands.CoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.commands.TestCommands.ToggleElevatorTest;
import frc.robot.commands.TestCommands.ToggleArmTest;
import frc.robot.subsystems.Climber.ClimberSubsystem;

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
                    // chassis.setDefaultCommand(chassis.run(() -> chassis.driveFieldCentric(driverController, DriveConstants.defaultDrivePower))); //Field centric init.
            
                    // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
                    // chassis will execute this command periodically
                    chassis.setDefaultCommand(
                        chassis.applyRequest(() -> drive
                                .withVelocityX(-driverController.getLeftY() * Math.abs(driverController.getLeftY()) * MaxSpeed * 0.9)
                                .withVelocityY(-driverController.getLeftX() * Math.abs(driverController.getLeftX()) * MaxSpeed * 0.9)
                                .withRotationalRate(-driverController.getRightX() * MaxAngularRate * 0.9)
                        ));
            
            
            
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
            
            
                    // /* ---------------------------------------- DRIVER CONTROLLER ----------------------------------------*/
                    // /* Sticks */  
                    // // Note that X is defined as forward according to WPILib convention,and Y is defined as to the left according to WPILib convention.
                    // driverController.leftStick().toggleOnTrue(new SemiAutoClimbCommand(Button.kLeftStick,Button.kRightStick));
            
                    // /* Bumpers & Triggers */
                    // driverController.rightBumper().whileTrue(superStructure.runOnce(() -> superStructure.getHybridCoralScoreCommand(Button.kRightBumper)));
                    // driverController.b().whileTrue(superStructure.runOnce(() -> superStructure.getHybridAlgaeScoreCommand(Button.kB,Button.kRightTrigger)));
                    //driverController.leftBumper().whileTrue(new ToggleIntake(grArm, intaker));
                    // driverController.leftBumper().whileTrue(superStructure.runOnce(() -> superStructure.getCoralAlignSequenceCommand(Button.kLeftBumper)));
                    // driverController.leftTrigger().whileTrue(superStructure.runOnce(() -> superStructure.getHybridAlgaeIntakeCommand(Button.kLeftTrigger)));
            
                    // /* Buttons */
                    
                    // driverController.b().whileTrue(chassis.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));
                    //driverController.povUp().onTrue(chassis.runOnce(() -> chassis.seedFieldCentric()));
                    // driverController.y().onTrue(superStructure.runOnce(() -> superStructure.getInitializationCommand(Button.kY)));
            
                    // /* Povs */
                    // operatorController.povUp().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(1)));
                    // operatorController.povDown().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefLevelIndex(-1)));
                    // operatorController.povLeft().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefPoseIndex(1)));
                    // operatorController.povRight().whileTrue(superStructure.runOnce(() -> superStructure.changeTargetReefPoseIndex(-1)));
            
                    // /* ---------------------------------------- OPERATOR CONTROLLER ----------------------------------------*/
            
                    // /* Sticks */
                    // /* Bumpers & Triggers */
                    operatorController.leftBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.LEFT)));
                    operatorController.rightBumper().onTrue(superStructure.runOnce(() -> superStructure.setDriverSelection(Selection.RIGHT)));
                    // operatorController.leftTrigger().onTrue(superStructure.runOnce(() -> superStructure.setTargetAlgaeIntakeLevelIndex(0)));
                    // operatorController.rightTrigger().onTrue(superStructure.runOnce(() -> superStructure.setTargetAlgaeIntakeLevelIndex(1)));
                    // /* Buttons */
                    operatorController.a().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(1)));
                    operatorController.b().onTrue(superStructure.runOnce(()-> superStructure.setTargetReefLevelIndex(2)));
                    operatorController.x().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(3)));
                    operatorController.y().onTrue(superStructure.runOnce(() -> superStructure.setTargetReefLevelIndex(4)));
                    
                    // /* Povs */
                    // operatorController.povUp().onTrue(superStructure.runOnce(() -> superStructure.setOperatorReefFaceIndex(6)));
                    // operatorController.povDown().onTrue(superStructure.runOnce(() -> superStructure.setOperatorReefFaceIndex(3)));
                    // operatorController.povLeft().onTrue(superStructure.runOnce(() -> superStructure.changeOperatorReefFaceIndex(Selection.LEFT)));
                    // operatorController.povRight().onTrue(superStructure.runOnce(() -> superStructure.changeOperatorReefFaceIndex(Selection.RIGHT)));
            
            
                    ///THOSE ARE FOR TESTING.-------------------------------------------------------------------------------------------    
                    //driverController.a().onTrue(new InstantCommand(()-> climber.setRotation(ClimberConstants.ClimberDefaultPos)));
                    //driverController.b().onTrue(new InstantCommand(()-> climber.setRotation(ClimberConstants.ClimberExtensionPos)));
                    //driverController.x().onTrue(new InstantCommand(()-> climber.setRotation(ClimberConstants.ClimberRetractionPos)));
                    driverController.x().onTrue(new InstantCommand(() -> chassis.resetPose(new Pose2d(0, 4, new Rotation2d()))));
                    driverController.back().onTrue(new NewClimbCommand(Button.kStart));
                    //driverController.x().onTrue(new InstantCommand(() -> chassis.resetPose(new Pose2d(0, 4, new Rotation2d()))));//This needs to be changed
            
                    //driverController.leftStick().toggleOnTrue(new InstantCommand(() -> climber.setPosition(1.)));//This needs to be changed
                    //driverController.leftStick().toggleOnFalse(new InstantCommand(() -> climber.setPosition(0.)));//This needs to be changed
            
                    //driverController.povLeft().onTrue(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("LBM-2").flipPath().getStartingHolonomicPose().get())));
            
                    // driverController.a().whileTrue(RobotContainer.chassis.followPPPath("1"));
                    // driverController.y().whileTrue(RobotContainer.chassis.followPPPath("2"));
                    
            
                    // driverController.leftTrigger().whileTrue(new InstantCommand(()-> shooter.setRPS(10)));
                    // driverController.rightTrigger().whileTrue(new InstantCommand(()->shooter.setRPS(-20)));
                    // driverController.rightBumper().whileTrue(new InstantCommand(()->shooter.setRPS(0)));
            
                    // driverController.x().onTrue(new InstantCommand(()->grArm.setPosition(90.)));
                    // driverController.y().onTrue(new InstantCommand(()->grArm.setPosition(-60.)));
                    // driverController.leftTrigger().onTrue(new InstantCommand(()->intaker.setRPS(10)));
                    // driverController.rightTrigger().onTrue(new InstantCommand(()->intaker.setRPS(-2)));
            
                    // driverController.y().whileTrue(new ToggleIntake(grArm, intaker));
            
            
            
                    driverController.rightBumper().onTrue(Commands.defer(()->superStructure.getNewCoralAlignSequenceCommand(Button.kA),Set.of(arm,elevator,shooter,indexer)));
                    driverController.rightBumper().whileTrue(Commands.defer(()->superStructure.getToggleIntakeCommand(),Set.of(grArm,intaker)));
            
            
                    // driverController.leftTrigger().whileTrue(Commands.defer(()->));
                    driverController.b().whileTrue(Commands.defer(()->superStructure.getManualAlgaeIntakeCommand(),Set.of(arm,elevator,shooter)));
                    driverController.a().onTrue(Commands.defer(()->superStructure.getManualAlgaeScoreCommand(),Set.of(elevator,shooter,arm)));
                    // driverController.a().whileTrue(RobotContainer.chassis.followPPPath("LBM-2"));
                    // driverController.b().whileTrue(RobotContainer.chassis.followPPPath("2-AlgaeLeft"));
                    // driverController.x().whileTrue(RobotContainer.chassis.followPPPath("AlgaeLeft-5"));
                
            
                    // driverController.y().whileTrue(Command.defer(()->));
                    // driverController.y().onTrue(new InstantCommand(()->indexer.setLeftRPS(-6.)));
                    // driverController.y().onFalse(new InstantCommand(()->indexer.setRPS(0)));
            
                    // driverController.a().onTrue(new InstantCommand(() -> indexer.setRPS(0)));
                    // driverController.x().onTrue(new InstantCommand(()->indexer.setRPS(2)));
                    // driverController.a().onTrue(new InstantCommand(()->indexer.setRPS(0)));
                    // driverController.a().onTrue(new InstantCommand(() -> intaker.setRPS(0)));
            
                    // driverController.leftBumper().onTrue(new InstantCommand(()->shooter.setRPS(-20)));
                    // driverController.leftBumper().onFalse(new InstantCommand(()->shooter.setRPS(0)));
                    // driverController.a().onTrue(new InstantCommand(()->chassis.resetPose(new Pose2d())));
                    // driverController.rightBumper().whileTrue(new InstantCommand(()->shooter.setRPS(20)));
                    // driverController.rightBumper().onFalse(new InstantCommand(()->shooter.setRPS(0)));
                    driverController.leftBumper().and(isNL1).whileTrue(Commands.defer(()->superStructure.getNewHybridCoralScoreCommand(Button.kRightTrigger),Set.of(arm,elevator,shooter,chassis)));
                    driverController.leftBumper().and(isL1).whileTrue(Commands.defer(()->superStructure.getCoralL1ScoreCommand(),Set.of(elevator,shooter,arm)));
                    driverController.povDown().onTrue(superStructure.runOnce(()->superStructure.forcelySetRobotStatus(RobotStatus.IDLE)));

        

        // driverController.rightBumper().onFalse(new InstantCommand(()->shooter.setRPS(0)));

        // driverController.povRight().onTrue(new InstantCommand(()->arm.setPosition(-90.)));
        // driverController.povLeft().onTrue(new InstantCommand(() -> arm.setPosition(-165.)));

        // driverController.povUp().onTrue(new InstantCommand(()->elevator.liftHeight(0.05)));
        // driverController.povDown().onTrue(new InstantCommand(()->elevator.liftHeight(-0.05)));

        // // driverController.y().onTrue(new InstantCommand(()->intaker.setRPS(-2)));
        //driverController.rightTrigger().onTrue(new InstantCommand(()->arm.setPosition(-90.)));
        //driverController.rightBumper().onTrue(new InstantCommand(()->arm.setPosition(-271.)));
        // // driverController.leftTrigger().whileTrue(new ZeroElevator());
        // // driverController.rightTrigger().whileTrue(new SetStateIdleDown());

        // driverController.a().onTrue(new InstantCommand(()->elevator.setHeight(-0.2)));//TODO MOVE INTO CONSTANTS

        // driverController.povRight().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.ElevatorHeights[1]));
        // driverController.povUp().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.ElevatorHeights[2]));
        // driverController.povLeft().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.ElevatorHeights[3]));
        // driverController.povDown().whileTrue(new ToggleElevatorTest(elevator,Constants.FieldConstants.ElevatorHeights[4]));
        //------------------------------------------------------------------------------------------------------------------
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
