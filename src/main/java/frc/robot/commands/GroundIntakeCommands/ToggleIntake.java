package frc.robot.commands.GroundIntakeCommands;

import frc.robot.Constants.GrArmConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();

    public ToggleIntake(GrArmSubsystem grArm, IntakerSubsystem intaker) {
        this.grArm = grArm;
        this.intaker = intaker;
        addRequirements(grArm, intaker);
    }

    @Override
    public void initialize() {
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
    }

    @Override
    public void end(boolean interrupted) {
        grArm.setPosition(GrArmConstants.RetractedPosition);
        intaker.setRPS(0);
        
    }

    public boolean isFinished() {
        return false; // This command runs until interrupted
    }

}