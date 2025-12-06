package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.constants.GeneralConstants.GrArmConstants;
import frc.robot.constants.GeneralConstants.IntakerConstants;
import frc.robot.containers.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public ToggleIntake(GrArmSubsystem grArm, IntakerSubsystem intaker) {
        this.grArm = grArm;
        this.intaker = intaker;
        addRequirements(grArm, intaker);
    }

    @Override
    public void initialize() {
        intaker.startIntake3D();//SIM
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
        
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
        intaker.stopIntake3D();//SIM
        grArm.setPosition(GrArmConstants.RetractedPosition);
        intaker.setRPS(0);
        
    }

    public boolean isFinished() {
        return false; // This command runs until interrupted
    }

}