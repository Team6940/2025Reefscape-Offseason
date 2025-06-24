package frc.robot.commands.AlgaeCommands;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

//WARNING: THIS COMMAND SHOULD NOT BE CALLED IF SHOOTER IS IN USE E.G SHOOTER READY TO SCORE CORAL
public class AlgaeHybridIntake extends Command {
    enum IntakeState {
        ALIGNING,
        PUSHING,
        INTAKING,
        DEPARTING,
        END
    }

    private int m_targetReefFaceIndex;
    private int m_targetAlgaeHeightIndex;
    private Button m_executionButton;
    IntakeState state;

    Pose2d targetPose;
    double targetHeight;
    double targetAngle;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public AlgaeHybridIntake(int targetReefFaceIndex, int targetAlgaeHeightIndex, Button executionButton) {
        addRequirements(elevator, shooter, chassis, arm);
        m_targetAlgaeHeightIndex = targetAlgaeHeightIndex;
        m_targetReefFaceIndex = targetReefFaceIndex;
        m_executionButton = executionButton;
    }

    @Override
    public void initialize() {
        state = IntakeState.ALIGNING;
        targetPose = chassis.generateAlgaePose(m_targetReefFaceIndex);
        targetHeight = FieldConstants.elevatorAlgaeIntakeHeight[m_targetAlgaeHeightIndex];
        targetAngle = FieldConstants.armIntakePosition[m_targetAlgaeHeightIndex];
        elevator.setHeight(0);
    }

    @Override
    public void execute() {
        chassis.hybridMoveToPose(targetPose, driverController,
                FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
        switch (state) {
            case ALIGNING:
                align();
                break;
            case PUSHING:
                push();
                break;
            case INTAKING:
                intake();
                break;
            case DEPARTING:
                depart();
                break;
            case END:
                break;
        }
    }

    public void align() {
        if (chassis.getToPoseDistance(targetPose) < FieldConstants.algaeAlignmentDistanceThreshold) {
            elevator.setHeight(targetHeight);
            arm.setPosition(targetAngle);
        }
        if (chassis.isAtTargetPose()) {
            state = IntakeState.PUSHING;
        }
    }

    public void push() {
        // Get current pose and add small forward offset (e.g. 0.2 meters)
        Pose2d currentPose = targetPose;
        Translation2d transformTranslation2d = new Translation2d(-FieldConstants.algaeIntakePushDistance,
                currentPose.getRotation());
        Pose2d pushPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
                currentPose.getRotation());
        // Move to push position
        chassis.autoMoveToPose(pushPose);

        if (chassis.isAtPose(pushPose) && driverController.getButton(m_executionButton)) {
            state = IntakeState.INTAKING;
        }
    }

    public void intake() {
        shooter.setRPS(ShooterConstants.AlgaeIntakingRPS);
        if (shooter.getShooterState() == ShooterState.READY) {
            state = IntakeState.DEPARTING;
        }
    }

    public void depart() {
        chassis.autoMoveToPose(targetPose);
        if (chassis.isAtPose(targetPose)) {
            arm.setPosition(FieldConstants.armAlgaeStowPosition);
            elevator.setHeight(0);
            shooter.stop();
            state = IntakeState.END;
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.stop();
        elevator.setHeight(0);
        shooter.stop();

    }
}
