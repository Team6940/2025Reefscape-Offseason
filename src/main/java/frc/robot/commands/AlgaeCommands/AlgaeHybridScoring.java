package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class AlgaeHybridScoring extends Command {
    enum ScoringState {
        ALIGNING,
        SCORING,
        DEPARTING,
        END
    }

    Pose2d currentPose;
    private double m_targetBargeLevel = Constants.FieldConstants.BargeHeight;
    private double m_targetBargeAngle = Constants.FieldConstants.BargeAngle;

    private Button m_executionButton;
    ScoringState state;

    Pose2d targetPose;
    double targetHeight;
    double targetAngle;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public AlgaeHybridScoring(Button executionButton) {
        addRequirements(elevator, shooter, chassis, arm);
        m_executionButton = executionButton;
    }

    @Override
    public void initialize() {
        state = ScoringState.ALIGNING;
        currentPose = chassis.getPose();
        targetPose = chassis.generateAlgaeScorePose(currentPose);
        targetHeight = m_targetBargeLevel;
        targetAngle = m_targetBargeAngle;
        elevator.setHeight(0);

    }

    @Override
    public void execute() {
        chassis.hybridMoveToPose(targetPose, driverController,
                FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
        SmartDashboard.putString("ALGAE hybrid Scoring State", state.toString());
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
        SmartDashboard.putString("ALGAE hybrid Scoring State", "ALIGNING");
        if (chassis.getToPoseDistance(targetPose) < FieldConstants.AlgaeScoreDistanceThreshold) {
            elevator.setHeight(targetHeight);
            arm.setPosition(targetAngle);
            if (arm.isAtTargetPositon() && elevator.isAtTargetHeight()) {
                state = ScoringState.SCORING;
                SmartDashboard.putString("ALGAE hybrid Scoring State", "ALIGNING complete, moving to PUSHING");
            }
        }
    }

    public void score() {
        SmartDashboard.putString("Hybrid Scoring State", "PUSHING");

        // Get current pose and add small forward offset (e.g. 0.2 meters)
        Pose2d currentPose = chassis.getPose();
        Translation2d transformTranslation2d = new Translation2d(-FieldConstants.AlgaeScorePushDistance,
                currentPose.getRotation());
        Pose2d pushPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
                currentPose.getRotation());
        // Move to push position
        chassis.autoMoveToPose(pushPose);
        shooter.setRPS(ShooterConstants.AlgaeScoringRPS); // TODO 'level' here indicates the fifth line of
                                                // ShoooterShootRPS(constants.java) ,which is the speed for shooting the
                                                // algae
        if (shooter.getShooterState() == ShooterState.FREE_SPINNING) {
            SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING complete, moving to DEPARTING");
            state = ScoringState.DEPARTING;
        } else {
            SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING in progress");
        }
    }

    public void depart() {
        SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING");

        chassis.autoMoveToPose(targetPose);  
        // When in position, reset systems and end
        if (chassis.isAtPose(targetPose)) {
            arm.setPosition(0);
            elevator.setHeight(0);
            shooter.stop();
            state = ScoringState.END;
            SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING complete, moving to END");
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("ALGAE hybrid Scoring State", "END due to interruption");
        arm.stop();
        elevator.setHeight(0);
        shooter.stop();
    }

}
