package frc.robot.commands.AlgaeCommands;

import java.util.concurrent.ExecutionException;

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
    private Button m_triggeringButton;

    ScoringState state;

    Pose2d targetPose;
    double targetHeight;
    double targetAngle;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public AlgaeHybridScoring(Button triggeringButton, Button executionButton) {
        addRequirements(elevator, shooter, chassis, arm);
        m_triggeringButton = triggeringButton;
        m_executionButton = executionButton;
    }

    @Override
    public void initialize() {
        state = ScoringState.ALIGNING;
        targetPose = chassis.generateAlgaeScorePose();
        targetHeight = m_targetBargeLevel;
        targetAngle = m_targetBargeAngle;
        elevator.setHeight(ElevatorConstants.IdleHeight);

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
            arm.setPosition(targetAngle);
            if (arm.isAtSecuredPosition()) {
                elevator.setHeight(targetHeight);
            }
            if (arm.isAtTargetPositon() && elevator.isAtTargetHeight()
                    && driverController.getButton(m_executionButton)) {
                state = ScoringState.SCORING;
                SmartDashboard.putString("ALGAE hybrid Scoring State", "ALIGNING complete, moving to PUSHING");
            }
        }
    }

    public void score() {
        SmartDashboard.putString("Hybrid Scoring State", "PUSHING");

        // Get current pose and add small forward offset (e.g. 0.2 meters)
        shooter.setRPS(ShooterConstants.AlgaeScoringRPS); // TODO 'level' here indicates the fifth line of
        // ShoooterShootRPS(constants.java) ,which is the speed for shooting the
        // algae
        if (!driverController.getButton(m_executionButton)) {
            SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING complete, moving to DEPARTING");
            state = ScoringState.DEPARTING;
        }
    }

    public void depart() {
        SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING");

       
        arm.setPosition(FieldConstants.ArmStowPosition);
        elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.stop();
        state = ScoringState.END;
        SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING complete, moving to END");
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("ALGAE hybrid Scoring State", "END due to interruption");
        arm.reset();
        elevator.setHeight(ElevatorConstants.IdleHeight);
        shooter.stop();
    }

}
