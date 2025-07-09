package frc.robot.commands.CoralCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UpperStructureState;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class NewCoralHybridScoring extends Command {
    enum ScoringState {
        ALIGNING,
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
    

    public NewCoralHybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button executionButton, boolean isReversed) {
        addRequirements(elevator, shooter, chassis, arm);
        m_targetReefPoseIndex = targetReefPoseIndex;
        m_targetReefLevelIndex = targetReefLevelIndex;
        m_executionButton = executionButton;
        m_isReversed=isReversed;
    }

    @Override
    public void initialize() {
        state = ScoringState.ALIGNING;
        targetPose = chassis.generateReefPose(m_targetReefPoseIndex);
        if(m_isReversed)
        {
            targetPose = chassis.generateReefPoseReversed(m_targetReefLevelIndex);
     aimHeight = Constants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).elevator_height;
        aimAngle = Constants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).arm_Angle;
        scoreHeight=Constants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).elevator_height;
        scoreAngle=Constants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).arm_Angle;
        targetRotation = FieldConstants.ReefRotationAdjustmentRange[m_targetReefLevelIndex];
        targetPose = chassis.generateReefPoseReversed(m_targetReefPoseIndex);
        }
        else{
            targetPose = chassis.generateReefPose(m_targetReefPoseIndex);
            aimHeight = Constants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).elevator_height;
               aimAngle = Constants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).arm_Angle;
               scoreHeight=Constants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).elevator_height;
               scoreAngle=Constants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).arm_Angle;
               targetRotation = FieldConstants.ReefRotationAdjustmentRange[m_targetReefLevelIndex];
               targetPose = chassis.generateReefPose(m_targetReefPoseIndex);

        }

    }

    @Override
    public void execute() {
        chassis.hybridMoveToPose(targetPose, driverController,
                FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
        SmartDashboard.putString("CORAL hybrid Scoring State", state.toString());
        switch (state) {
            case ALIGNING:
                align();
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

    public void align() {
        elevator.setHeight(aimHeight);
        arm.setPosition(aimAngle);
        if (arm.isAtTargetPositon()) {
            state = ScoringState.SCORING;
        }
    }

    public void score() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING");
        arm.setPosition(scoreAngle);
        elevator.setHeight(scoreHeight);
        if (shooter.getShooterState() == ShooterState.IDLE) {
            SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING complete, moving to DEPARTING");
            state = ScoringState.DEPARTING;
        } else {
            SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING in progress");
        }

    }

    public void depart() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING");

        // Calculate retreat position (move backward)
        Pose2d currentPose = targetPose;
        Translation2d transformTranslation2d = new Translation2d(FieldConstants.CoralScorePushDistance,
                currentPose.getRotation());
        Pose2d departPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
                currentPose.getRotation());

        // Move to retreat position
        targetPose = departPose;
        chassis.autoMoveToPose(targetPose);

        shooter.setRPS(ShooterConstants.CoralScoringRPS);
        // When in position, reset systems and end
        if (shooter.getShooterState() == ShooterState.IDLE) {
            arm.setPosition(UpperStructureState.IdleDown.arm_Angle);
            elevator.setHeight(UpperStructureState.IdleDown.elevator_height);
            shooter.stop();
            state = ScoringState.END;
            SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING complete, moving to END");
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("CORAL hybrid Scoring State", "END due to interruption");
            arm.setPosition(UpperStructureState.IdleDown.arm_Angle);
            elevator.setHeight(UpperStructureState.IdleDown.elevator_height);
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
