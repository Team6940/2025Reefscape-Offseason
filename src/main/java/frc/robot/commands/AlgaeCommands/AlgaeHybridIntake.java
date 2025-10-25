package frc.robot.commands.AlgaeCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.constants.GeneralConstants.ShooterConstants;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.RobotStatus;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

//WARNING: THIS COMMAND SHOULD NOT BE CALLED IF SHOOTER IS IN USE E.G SHOOTER READY TO SCORE CORAL
public class AlgaeHybridIntake extends Command {
    enum IntakeState {
        ALIGNING,
        PREPARING,
        PUSHING,
        DEPARTING,
        RETRACTING,
        END
    }

    private int m_targetReefFaceIndex;
    private Button m_executionButton;
        SuperStructure superStructure = SuperStructure.getInstance();
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
        targetPose = chassis.generateAlgaeIntakePose(m_targetReefFaceIndex);
        targetHeight = FieldConstants.ElevatorAlgaeIntakeHeight[m_targetReefFaceIndex];
        targetAngle = FieldConstants.ArmIntakePosition[m_targetReefFaceIndex];
        shooter.setRPS(ShooterConstants.AlgaeIntakingRPS);
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
            case PREPARING:
                prepare();
                break;
            case PUSHING:
                push();
                break;
            case DEPARTING:
                depart();
                break;
            case RETRACTING:
                retract();
                break;
            case END:
                break;
        }
    }

    public void align() {
        if (elevator.isAtTargetHeight()) {
            state=IntakeState.PREPARING;
        }
    }

    public void prepare() {
        arm.setPosition(targetAngle);
        elevator.setHeight(targetHeight);
        if(arm.isAtTargetPositon()&&chassis.isAtPose(targetPose)){
            Transform2d pushTransform2d = new Transform2d(-FieldConstants.AlgaeIntakePushDistance, 0, Rotation2d.kZero);
            Pose2d pushPose = targetPose.plus(pushTransform2d);
            targetPose = pushPose; // Update target pose to the push position
            state=IntakeState.PUSHING;
        }
    }

    public void push() {
        if (chassis.isAtPose(targetPose)) {
            Transform2d departTransform2d = new Transform2d(FieldConstants.AlgaeIntakePushDistance, 0, Rotation2d.kZero);
            Pose2d departPose = targetPose.plus(departTransform2d);
            targetPose = departPose; // Update target pose to the depart position
            state = IntakeState.DEPARTING;
        }
    }

    public void depart() {
        if (chassis.isAtPose(targetPose)) {
           state= IntakeState.RETRACTING;
        }
    }

    public void retract() {
        arm.setPosition(FieldConstants.ArmStowPosition);
        elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.setRPS(-7.);
        state = IntakeState.END;
    }

    @Override
    public void end(boolean interrupted) {
       elevator.setHeight(ElevatorConstants.MinHeight);
        arm.setPosition(FieldConstants.ArmStowPosition);
        superStructure.forcelySetRobotStatus(RobotStatus.HOLDING_ALGAE);
        shooter.setRPS(-7.);
    }
}
