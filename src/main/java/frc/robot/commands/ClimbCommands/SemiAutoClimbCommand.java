package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;

public class SemiAutoClimbCommand extends Command{

    ClimberSubsystem climber;
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    ImprovedCommandXboxController controller = new ImprovedCommandXboxController(0);

    Button toggleButton; //toggle button also act as a release button (toggle 2nd)
    Button retractButton;

    Translation2d pushTransform = new Translation2d(
        DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.ClimbPushDis : -FieldConstants.ClimbPushDis,
        0
    );
    Translation2d retreatTransform = new Translation2d(
        DriverStation.getAlliance().get() == Alliance.Blue ? FieldConstants.ClimbRetreatToDis : -FieldConstants.ClimbRetreatToDis,
        0
    );
    Pose2d poseToPush = new Pose2d();
    Pose2d poseToRetreat = new Pose2d();

    public SemiAutoClimbCommand(Button toggleButton,Button retractButton) {
        this.toggleButton = toggleButton;
        this.retractButton = retractButton;
        climber = ClimberSubsystem.getInstance();
        addRequirements(climber);
        addRequirements(chassis);
        addRequirements(elevator);
        addRequirements(arm);
    }

    public enum ClimbState {
        EXTENDING,
        PUSHING,
        RETRACTING,
        RETRACTED,
    }
    
    ClimbState state;

    @Override
    public void initialize() {
        state = ClimbState.EXTENDING;
    }

    @Override
    public void execute(){
        switch (state) {
            case EXTENDING:
                extend();
                break;
            case PUSHING:
                push();
                break;
            case RETRACTING:
                retract();
                break; 
            case RETRACTED:
                break;
            default:
                break;
        }
    }

    private void extend() {
        climber.setPosition(ClimberConstants.ClimberExtensionPos);
        arm.setPosition(FieldConstants.armAngles[4]); //for keeping balance while climbing //TODO set height / clockwise or conter_clockwise
        chassis.driveFieldCentric(controller, DriveConstants.defaultDrivePower);
        if(climber.isAtTargetRotation() && controller.getButton(toggleButton)){
        // pushTransform=pushTransform.rotateBy(Rotation2d.k180deg);//2025.3.14 rotatethe push and pull transform
        // retreatTransform=pushTransform.rotateBy(Rotation2d.k180deg);
            poseToPush = new Pose2d(
                    chassis.getPose().getTranslation().plus(pushTransform),
                    DriverStation.getAlliance().get() == Alliance.Red? Rotation2d.kZero : Rotation2d.k180deg//2025.3.14
            );
            poseToRetreat = new Pose2d(
                    chassis.getPose().getTranslation().plus(retreatTransform),
                    DriverStation.getAlliance().get() == Alliance.Red? Rotation2d.kZero : Rotation2d.k180deg 
            );
            state = ClimbState.PUSHING;
        }
    }

    private void push(){
        chassis.hybridMoveToPose(poseToPush, controller, 0.15, 15.); //TODO need to change position
        climber.lockMotorSetRPS(ClimberConstants.LockMotorRPS);
        if( controller.getButton(retractButton) ){ //if retract button is pressed, retract
            state = ClimbState.RETRACTING;
        };
    }

    private void retract() {
        climber.setPosition(ClimberConstants.ClimberRetractionPos);
        // elevator.setHeight(FieldConstants.elevatorClimbHeight);
        chassis.hybridMoveToPose(poseToRetreat, controller, 0.24, 15.);
        if(climber.isAtTargetRotation()){
            chassis.brake();
            state = ClimbState.RETRACTED;
        }
    }

    @Override
    public void end(boolean interrupted) {          //this shouldn't be called if climbed successfully
        chassis.brake();
        // climber.setPosition(ClimberConstants.ClimberExtensionPos);
        climber.setPosition(ClimberConstants.ClimberDefaultPos);
        climber.lockMotorSetRPS(0.0); //TODO: stop lock motor
        // elevator.setHeight(0);
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isDisabled()) return true;
        return false;
    }
}
