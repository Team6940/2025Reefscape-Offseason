package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoPreparation;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.AlgaeCommands.AlgaeHybridScoring;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;

public class LeftSide extends SequentialCommandGroup {
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();

    public LeftSide() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            addCommands(new InstantCommand(
                    () -> chassis.resetPose(chassis.generatePPPath("LBL-Front").getStartingHolonomicPose().get())));
        } else {
            addCommands(new InstantCommand(() -> chassis
                    .resetPose(chassis.generatePPPath("LBL-Front").flipPath().getStartingHolonomicPose().get())));
        } // set original pose

        addCommands(chassis.followPPPath("LBL-Front").alongWith(new AutoPreparation()));
    }
}
