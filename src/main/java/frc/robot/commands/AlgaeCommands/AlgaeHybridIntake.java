package frc.robot.commands.AlgaeCommands;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
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

    public AlgaeHybridIntake(int targetReefFaceIndex, Button executionButton) {
        addRequirements(elevator, shooter, chassis, arm);
        m_targetReefFaceIndex = targetReefFaceIndex;
        m_executionButton = executionButton;
    }

    @Override
    public void initialize() {
        state = IntakeState.ALIGNING;
        targetPose = chassis.generateAlgaeIntakePose(m_targetReefFaceIndex);
        targetHeight = FieldConstants.ElevatorAlgaeIntakeHeight[m_targetReefFaceIndex];
        targetAngle = FieldConstants.ArmIntakePosition[m_targetReefFaceIndex];
        elevator.setHeight(ElevatorConstants.IdleHeight);
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
        if (chassis.getToPoseDistance(targetPose) < FieldConstants.AlgaeAlignmentDistanceThreshold) {
            arm.setPosition(targetAngle);
            if(arm.isAtSecuredPosition()){
                elevator.setHeight(targetHeight);
            }
        }
        if (chassis.isAtTargetPose()) {
            state = IntakeState.PUSHING;
        }
    }

    public void push() {
        // Get current pose and add small forward offset (e.g. 0.2 meters)
        Pose2d currentPose = targetPose;
        Translation2d transformTranslation2d = new Translation2d(-FieldConstants.AlgaeIntakePushDistance,
                currentPose.getRotation());
        Pose2d pushPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
                currentPose.getRotation());
        // Move to push position
        shooter.setRPS(ShooterConstants.AlgaeIntakingRPS);
        chassis.autoMoveToPose(pushPose);

        if (chassis.isAtPose(pushPose)) {
            state = IntakeState.INTAKING;
        }
    }

    public void intake() {
        if (shooter.getShooterState() == ShooterState.READY) {
            state = IntakeState.DEPARTING;
        }
    }

    public void depart() {
        shooter.setRPS(ShooterConstants.HoldingAlgaeRPS); //get hold of the coral in case the robot throws it out accidently
        chassis.autoMoveToPose(targetPose);
        if (chassis.isAtPose(targetPose)) {
            arm.setPosition(FieldConstants.ArmStowPosition);
            elevator.setHeight(ElevatorConstants.IdleHeight);
            // shooter.stop();
            state = IntakeState.END;
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.reset();
        elevator.setHeight(ElevatorConstants.IdleHeight);
        shooter.stop();
    }
}
