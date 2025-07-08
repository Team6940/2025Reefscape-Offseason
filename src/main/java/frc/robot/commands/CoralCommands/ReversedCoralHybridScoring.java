package frc.robot.commands.CoralCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class ReversedCoralHybridScoring extends Command {
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
    double targetHeight;
    double targetAngle;
    double targetRotation;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public ReversedCoralHybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button executionButton) {
        addRequirements(elevator, shooter, chassis, arm, grArm);
        m_targetReefPoseIndex = targetReefPoseIndex;
        m_targetReefLevelIndex = targetReefLevelIndex;
        m_executionButton = executionButton;
    }

    @Override
    public void initialize() {
        state = ScoringState.ALIGNING;
        targetPose = chassis.generateReefPoseReversed(m_targetReefPoseIndex);
        targetHeight = FieldConstants.ElevatorHeightsReversed[m_targetReefLevelIndex];
        targetAngle = FieldConstants.ArmAnglesReversed[m_targetReefLevelIndex];
        targetRotation = FieldConstants.ReefRotationAdjustmentRangeReversed[m_targetReefLevelIndex];
        elevator.setHeight(ElevatorConstants.IdleHeight);
    }

    @Override
    public void execute() {
        chassis.hybridMoveToPose(targetPose, driverController, FieldConstants.reefTranslationAdjustmentRange,
                FieldConstants.reefRotationAdjustmentRangeDegs);
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
        elevator.setHeight(targetHeight);
        arm.setPosition(targetAngle);
        if (arm.isAtTargetPositon()) {
            state = ScoringState.SCORING;
        }
    }

    public void score() {
        arm.rotateArm(targetAngle);
    }

    public void depart() {
        shooter.setRPS(ShooterConstants.CoralScoringRPS);

        Pose2d currentPose = chassis.getPose();
        Translation2d transformTranslation2d = new Translation2d(FieldConstants.CoralScorePushDistance,
                currentPose.getRotation());
        Pose2d departPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
                currentPose.getRotation());
        chassis.autoMoveToPose(departPose);// Move to retreat position

        if (shooter.getShooterState() == ShooterState.IDLE) {
            arm.reset();
            elevator.setHeight(ElevatorConstants.IdleHeight);
            shooter.stop();
            state = ScoringState.END;
            SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING complete, moving to END");
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.reset();
        elevator.setHeight(ElevatorConstants.IdleHeight);
        shooter.stop();
    }

    public CoralHybridScoring withSelection(Selection selection) {
        switch (selection) {
            case LEFT:
                return new CoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 1, m_targetReefLevelIndex,
                        m_executionButton);
            case RIGHT:
                return new CoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 2, m_targetReefLevelIndex,
                        m_executionButton);

            default:
                return null;
        }
    }

}
