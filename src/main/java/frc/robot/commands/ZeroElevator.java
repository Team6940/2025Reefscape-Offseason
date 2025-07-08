package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrArmConstants;
import frc.robot.Constants.UpperStructureState;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;

public class ZeroElevator extends Command{
    ArmSubsystem arm = ArmSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();

    public ZeroElevator() {
        addRequirements(arm,grArm, elevator);
    }

    @Override
    public void initialize() {
        grArm.setPosition(GrArmConstants.ExtendedPosition);
    }

    @Override
    public void execute() {
        elevator.setVoltage(-0.5);
        arm.setPosition(90);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setVoltage(0.);
        elevator.resetHeight(0);
    }
}
