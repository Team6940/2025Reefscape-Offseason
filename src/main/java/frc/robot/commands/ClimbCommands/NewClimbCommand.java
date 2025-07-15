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

public class NewClimbCommand extends Command{

    ClimberSubsystem climber;
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    ImprovedCommandXboxController controller = new ImprovedCommandXboxController(0);

    Button toggleButton;

    public NewClimbCommand(Button toggleButton) {
        this.toggleButton = toggleButton;
        climber = ClimberSubsystem.getInstance();
        addRequirements(climber);
        // addRequirements(chassis);
        addRequirements(elevator);
        addRequirements(arm);
    }

    public enum ClimbState {
        EXTENDING,
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
        climber.setPosition(ClimberConstants.ClimberExtensionPos);
        // elevator.setHeight(FieldConstants.ElevatorClimbHeight); //for keeping balance
        // arm.setPosition(FieldConstants.ArmClimbPositionDegs); //for keeping balance while climbing //TODO set height / clockwise or conter_clockwise
        // chassis.driveFieldCentric(controller, DriveConstants.defaultDrivePower);
        if(climber.isAtTargetRotation() && controller.getButton(toggleButton)){ //TODO
            state = ClimbState.RETRACTING;
        }
    }

    private void retract() {
        climber.setPosition(ClimberConstants.ClimberRetractionPos);
        // elevator.setHeight(FieldConstants.elevatorClimbHeight);
        if(climber.isAtTargetRotation()){
            // chassis.brake();
            state = ClimbState.DONE;
        }
    }

    @Override
    public void end(boolean interrupted) {          //this shouldn't be called if climbed successfully
        // chassis.brake();
        climber.setPosition(ClimberConstants.ClimberDefaultPos);
        // climber.lockMotorSetRPS(0.0); //TODO: stop lock motor
        // elevator.setHeight(0);
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isDisabled()) return true;
        return false;
    }
}
