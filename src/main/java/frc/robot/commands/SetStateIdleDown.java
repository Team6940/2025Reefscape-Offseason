package frc.robot.commands;

import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UpperStructureState;

public class SetStateIdleDown extends Command{
    ArmSubsystem arm = ArmSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    
    public SetStateIdleDown() {
        addRequirements(arm, elevator);
    }

    public void initialize() {
        arm.setPosition(UpperStructureState.IdleDown.arm_Angle);
        elevator.setHeight(UpperStructureState.IdleDown.elevator_height);
    }
}
