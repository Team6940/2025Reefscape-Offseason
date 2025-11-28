package frc.robot.commands.CoralCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.constants.GeneralConstants.ShooterConstants;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class NewCoralHybridScoring extends Command {
    enum ScoringState {
        AIMING,
        SCORING,
        DEPARTING,
        END
    }

    private int m_targetReefPoseIndex;
    private int m_targetReefLevelIndex;
    private Button m_executionButton;
    ScoringState state;

    Pose2d targetPose;
    double aimHeight;
    double aimAngle;
    double scoreHeight;
    double scoreAngle;
    boolean m_isReversed;
    double targetRotation;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double driveDeadband = 0.045;
    public static final double rotateDeadband = 0.045;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * driveDeadband).withRotationalDeadband(MaxAngularRate * rotateDeadband)
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive 

    public NewCoralHybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button executionButton, boolean isReversed) {
        addRequirements(elevator, shooter, chassis, arm);
        m_targetReefPoseIndex = targetReefPoseIndex;
        m_targetReefLevelIndex = targetReefLevelIndex;
        m_executionButton = executionButton;
        m_isReversed=isReversed;
    }

    @Override
    public void initialize() {
        state = ScoringState.AIMING;
        // if(m_targetReefLevelIndex<=1)
        if(m_isReversed)
        {
            aimHeight = GeneralConstants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            aimAngle = GeneralConstants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).armAngleDegs;
            scoreHeight=GeneralConstants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            scoreAngle=GeneralConstants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).armAngleDegs;
            targetPose = chassis.generateReefPoseReversed(m_targetReefPoseIndex,m_targetReefLevelIndex);
        }
        else{
            aimHeight = GeneralConstants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            aimAngle = GeneralConstants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).armAngleDegs;
            scoreHeight=GeneralConstants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            scoreAngle=GeneralConstants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).armAngleDegs;
            targetPose = chassis.generateReefPose(m_targetReefPoseIndex,m_targetReefLevelIndex);

        }

    }

    @Override
    public void execute() {
        if(m_targetReefLevelIndex>=2)
            chassis.hybridMoveToPose(targetPose, driverController,
                FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
        SmartDashboard.putString("CORAL hybrid Scoring State", state.toString());
        switch (state) {
            case AIMING:
                aim();
                break;
            case SCORING:
                score();
                break;
            case DEPARTING:
                depart();
                break;
            case END:
                break;
        }
    }

    public void aim() {
        elevator.setHeight(aimHeight);
        arm.setPosition(aimAngle);
        if (
            driverController.getButton(Button.kLeftTrigger)
        ) 
        {
            state = ScoringState.SCORING;
        }
    
    }

    public void score() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING");
        if(m_targetReefLevelIndex>=2){
        arm.setPosition(scoreAngle);
        elevator.setHeight(scoreHeight);
        if (arm.isAtTargetPositon() && elevator.isAtTargetHeight()) {
            SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING complete, moving to DEPARTING");
            state = ScoringState.DEPARTING;
            Transform2d retreatTransform2d = new Transform2d(FieldConstants.CoralScoreRetreatDistance, 0, Rotation2d.kZero);
            Pose2d departPose = targetPose.plus(retreatTransform2d);
            targetPose = departPose;
        } else {
            SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING in progress");
        }}
        else
        {
            shooter.setRPS(ShooterConstants.CoralScoringRPS);
        }

    }

    public void depart() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING");

        // Calculate retreat position (move backward)
        // Pose2d currentPose = targetPose;
        // Translation2d transformTranslation2d = new Translation2d(FieldConstants.CoralScorePushDistance,
        //         currentPose.getRotation());
        // Pose2d departPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
        //         currentPose.getRotation());


        // Move to retreat position
        shooter.setRPS(ShooterConstants.CoralScoringRPS);

        // When in position, reset systems and end
        // if (chassis.isAtTargetPose()) {
        //     arm.setPosition(FieldConstants.ArmStowPosition);
        //     elevator.setHeight(ElevatorConstants.MinHeight);
        //     shooter.stop();
        //     state = ScoringState.END;
        //     SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING complete, moving to END");
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("CORAL hybrid Scoring State", "END due to interruption");
            arm.setPosition(FieldConstants.ArmStowPosition);
            elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.stop();
    }

    public NewCoralHybridScoring withSelection(Selection selection) {
        switch (selection) {
            case LEFT:
                return new NewCoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 1, m_targetReefLevelIndex,
                        m_executionButton, m_isReversed);
            case RIGHT:
                return new NewCoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 2, m_targetReefLevelIndex,
                        m_executionButton, m_isReversed);

            default:
                return null;
        }
    }

}
