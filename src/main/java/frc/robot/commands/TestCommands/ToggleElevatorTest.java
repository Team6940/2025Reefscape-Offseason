package frc.robot.commands.TestCommands;

import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleElevatorTest extends Command {
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    double position;

    public ToggleElevatorTest(ElevatorSubsystem elevator,Double postion) {
        this.elevator= elevator;
        this.position = postion;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(position);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setHeight(ElevatorConstants.IdleHeight); // Reset to idle height when command ends
    }

    public boolean isFinished(){
        return false; // This command runs until interrupted
    }
    

}