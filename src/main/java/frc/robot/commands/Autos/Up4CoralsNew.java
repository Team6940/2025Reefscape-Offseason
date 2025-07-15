package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.commands.GroundIntakeCommands.AutoIntakeCoral;
// import frc.robot.commands.IntakeCommands.CentreCoralPos;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake; //TODO
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.commands.CoralCommands.NewCoralHybridScoring;

public class Up4CoralsNew extends SequentialCommandGroup {
    // TODO: it's possible to use HybridCommands directly instead of FollowPath in
    // order to save time
    // TODO: This needs to be determined after tests
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Up4CoralsNew() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            addCommands(new InstantCommand(
                    () -> chassis.resetPose(chassis.generatePPPath("LBM-2").getStartingHolonomicPose().get())));
        } else {
            addCommands(new InstantCommand(() -> chassis
                    .resetPose(chassis.generatePPPath("LBM-2").flipPath().getStartingHolonomicPose().get())));
        } // set the orignal pose

        addCommands(chassis.followPPPath("LBM-2"));
        addCommands(new NewCoralHybridScoring(2, 4, Button.kAutoButton, true).withTimeout(1));// score 2

        addCommands(chassis.followPPPath("2-AlgaeLeft").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("AlgaeLeft-5"));
        addCommands(new NewCoralHybridScoring(5, 4, Button.kAutoButton, true).withTimeout(1));// score 4

        addCommands(chassis.followPPPath("5-AlgaeMid").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("AlgaeMid-6"));
        addCommands(new NewCoralHybridScoring(6, 4, Button.kAutoButton, true).withTimeout(1));// score 3

        addCommands(chassis.followPPPath("6-AlgaeRight").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("AlgaeRight-7"));
        addCommands(new NewCoralHybridScoring(7, 4, Button.kAutoButton, true).withTimeout(1));// score 5
    }
}
