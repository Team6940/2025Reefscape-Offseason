package frc.robot.commands.GroundIntakeCommands;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GrArmConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.commands.Rumble;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleIntake extends Command {
    Pose2d autoIntakePose;
    boolean ltHeldLastCycle = false;

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
        intaker.startIntake3D();//SIM
        grArm.setPosition(GrArmConstants.ExtendedPosition);
        intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
    }

    @Override
    public void execute() {
        boolean ltHeld = driverController.getButton(Button.kLeftTrigger);
        // ----------------------------------------------
        // 1) LT JUST PRESSED → capture & freeze pose
        // ----------------------------------------------
        if (ltHeld && !ltHeldLastCycle) { // rising edges
            if (vision.hasValidTarget()) {
                autoIntakePose = vision.getObjectFieldRelativePose2d(
                        vision.getLowestObject(0.),
                        chassis.getPose());
            } else {
                autoIntakePose = null;
            }
        }

        // ----------------------------------------------
        // 2) Rumble if valid target
        // ----------------------------------------------
        if (vision.hasValidTarget()) {
            new Rumble(RumbleType.kBothRumble, 1.).withTimeout(0.02).schedule();
        }

        // ----------------------------------------------
        // 3) If LT held and pose frozen → auto-align
        // ----------------------------------------------
        if (ltHeld && autoIntakePose != null) {
            chassis.hybridMoveToPose(autoIntakePose, driverController, 0.5, 20);
        } else {
            chassis.driveFieldCentric(driverController, 1);
        }

        // ----------------------------------------------
        // 4) Update LT history
        // ----------------------------------------------
        ltHeldLastCycle = ltHeld;

        if (driverController.getButton(Button.kA)) {
            intaker.setRPS(-20);
        } else {
            intaker.setRPS(IntakerConstants.IntakerIntakingRPS);
        } // this decides whether to run the intaker in or out

        Logger.recordOutput("ToggleIntakeAutoPose", autoIntakePose);
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