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
    BooleanSupplier toggleButton;

    public ToggleIntake(GrArmSubsystem grArm, IntakerSubsystem intaker, BooleanSupplier toggleButton) {
        this.grArm = grArm;
        this.intaker = intaker;
        this.toggleButton = toggleButton;
        addRequirements(grArm, intaker);
    }

    @Override
    public void execute() {
        if (toggleButton.getAsBoolean()) {
            extend();
        } else {
            retract();
        }
    }

    private void extend(){
        grArm.setPosition(GrArmConstants.extendedPosition);
        intaker.setRPS(IntakerConstants.intakingRPS);
    }

    private void retract(){
        grArm.setPosition(GrArmConstants.retractedPosition);
        intaker.setRPS(0);
    }

    public boolean isFinished(){
        return false; // This command runs until interrupted
    }
    

}