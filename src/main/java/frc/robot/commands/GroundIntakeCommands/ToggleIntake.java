package frc.robot.commands.GroundIntakeCommands;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GrArmConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.commands.Rumble;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    Pose2d autoIntakePose;
    boolean isAutoIntakingAvailable = false;

    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    VisionSubsystem vision = VisionSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public ToggleIntake(GrArmSubsystem grArm, IntakerSubsystem intaker) {
        this.grArm = grArm;
        this.intaker = intaker;
        addRequirements(grArm, intaker, chassis);
    }

    @Override
    public void initialize() {
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
        //indexer.setLeftRPS(-5.);
        //indexer.setRghtRPS(-10);
        
        if(vision.hasValidTarget()){
            autoIntakePose = vision.getObjectFieldRelativePose2d(vision.getPrimaryObject(), chassis.getPose());
            isAutoIntakingAvailable = true;
            new Rumble(RumbleType.kBothRumble, 1.).withTimeout(0.3).schedule();
        }
    }

    @Override
    public void execute() {
        if(isAutoIntakingAvailable && driverController.getButton(Button.kLeftTrigger)){
            chassis.hybridMoveToPose(autoIntakePose, driverController,0.2, 20);
        }
        else{
            chassis.driveFieldCentric(driverController,1);
        }

        if(driverController.getButton(Button.kA)){
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