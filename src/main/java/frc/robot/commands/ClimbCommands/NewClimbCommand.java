package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.constants.GeneralConstants.ClimberConstants;
import frc.robot.constants.GeneralConstants.DriveConstants;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.constants.GeneralConstants.GrArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;

public class NewClimbCommand extends Command{

    ClimberSubsystem climber = ClimberSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    ImprovedCommandXboxController controller = new ImprovedCommandXboxController(0);

    Button toggleButton;

    public NewClimbCommand(Button toggleButton) {
        this.toggleButton = toggleButton;
        climber = ClimberSubsystem.getInstance();
        addRequirements(climber);
        // addRequirements(chassis);
        addRequirements(elevator);
        
        addRequirements(arm);
        addRequirements(grArm);
    }

    public enum ClimbState {
        EXTENDING,
        DROPPING,
        RETRACTING,
        DONE
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
            case DROPPING:
                drop();
                break;
            case RETRACTING:
                retract();
                break; 
            case DONE:
                break;
            default:
                break;
        }
    }

    private void extend() {
        climber.setRotation(ClimberConstants.ClimberExtensionPos);
        elevator.setHeight(ElevatorConstants.IdleHeight);
        // elevator.setHeight(FieldConstants.ElevatorClimbHeight); //for keeping balance
        arm.setPosition(FieldConstants.ArmClimbPositionDegs); //for keeping balance while climbing //TODO set height / clockwise or conter_clockwise
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        // chassis.driveFieldCentric(controller, DriveConstants.defaultDrivePower);
        if(arm.isAtTargetPositon()){ //TODO
            state = ClimbState.DROPPING;
        }
    }

    private void drop(){
        elevator.setHeight(ElevatorConstants.MinHeight);
        if(elevator.isAtTargetHeight()&&controller.getButton(toggleButton)) {
            state = ClimbState.RETRACTING;
        }
    }

    private void retract() {
        climber.setRotation(ClimberConstants.ClimberRetractionPos);
        // elevator.setHeight(FieldConstants.elevatorClimbHeight);
        if(climber.isAtTargetRotation()){
            // chassis.brake();
            state = ClimbState.DONE;
        }
    }

    @Override
    public void end(boolean interrupted) {          //this shouldn't be called if climbed successfully
        // chassis.brake();
        if(interrupted){
            climber.setRotation(ClimberConstants.ClimberDefaultPos);
            arm.setPosition(FieldConstants.ArmStowPosition);
            elevator.setHeight(ElevatorConstants.MaxHeight);
            grArm.setPosition(GrArmConstants.RetractedPosition);
        }
        // climber.lockMotorSetRPS(0.0); //TODO: stop lock motor
        // elevator.setHeight(0);
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isDisabled()) return true;
        return false;
    }
}
