package frc.robot.commands.TestCommands;

import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;

public class ToggleArmTest extends Command {
    ArmSubsystem arm = ArmSubsystem.getInstance();
    double position;

    public ToggleArmTest(ArmSubsystem arm, Double position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPosition(-90.); // Reset to idle height when command ends
    }

    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}
