package frc.robot.commands.GroundIntakeCommands;

import frc.robot.Constants.GrArmConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public ToggleIntake(GrArmSubsystem grArm, IntakerSubsystem intaker) {
        this.grArm = grArm;
        this.intaker = intaker;
        addRequirements(grArm, intaker);
    }

    @Override
    public void initialize() {
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
        //indexer.setLeftRPS(-5.);
        //indexer.setRghtRPS(-10);
        
    }

    @Override
    public void execute() {
        if(driverController.getButton(Button.kRightTrigger)){
            intaker.setRPS(-20);
        }
        else{
            intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
        }
    }

    @Override
    public void end(boolean interrupted) {
        grArm.setPosition(GrArmConstants.RetractedPosition);
        intaker.setRPS(0);
        //indexer.stop();
        
    }

    public boolean isFinished() {
        return false; // This command runs until interrupted
    }

}