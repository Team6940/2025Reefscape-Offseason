package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoPreparation;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.commands.GroundIntakeCommands.AutoIntakeCoral;
// import frc.robot.commands.IntakeCommands.CentreCoralPos;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake; //TODO
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.commands.CoralCommands.NewCoralHybridScoring;

public class Down4Corals extends SequentialCommandGroup {
    // TODO: it's possible to use HybridCommands directly instead of FollowPath in
    // order to save time
    // TODO: This needs to be determined after tests

    // TODO: Some FollowPath Poses needs to be carefully calculated to make sure
    // it is exactly the same as what CoralHybridScoring calculates

    // TODO: Some Paths' contraints should be removed or modified to make the bot
    // run faster
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Down4Corals() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            addCommands(new InstantCommand(
                    () -> chassis.resetPose(chassis.generatePPPath("RBM-9").getStartingHolonomicPose().get())));
        } else {
            addCommands(new InstantCommand(() -> chassis
                    .resetPose(chassis.generatePPPath("RBM-9").flipPath().getStartingHolonomicPose().get())));
        } // set the orignal pose

        addCommands(chassis.followPPPath("RBM-9").alongWith(new AutoPreparation()));
        addCommands(new NewCoralHybridScoring(9, 4, Button.kAutoButton,true).withTimeout(2));// score 9

        addCommands(chassis.followPPPath("9-RSR").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("RSR-7"));
        addCommands(new NewCoralHybridScoring(7, 4, Button.kAutoButton,true).withTimeout(2));// score 7

        addCommands(chassis.followPPPath("7-RSR").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("RSR-8"));
        addCommands(new NewCoralHybridScoring(8, 4, Button.kAutoButton,true).withTimeout(2));// score 8

    }

}
