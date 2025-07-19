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

public class Up4Corals extends SequentialCommandGroup {
    // TODO: it's possible to use HybridCommands directly instead of FollowPath in
    // order to save time
    // TODO: This needs to be determined after tests
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Up4Corals() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            addCommands(new InstantCommand(
                    () -> chassis.resetPose(chassis.generatePPPath("LBM-2").getStartingHolonomicPose().get())));
        } else {
            addCommands(new InstantCommand(() -> chassis
                    .resetPose(chassis.generatePPPath("LBM-2").flipPath().getStartingHolonomicPose().get())));
        } // set the orignal pose

        addCommands(chassis.followPPPath("LBM-2").alongWith(new AutoPreparation()));
        addCommands(new NewCoralHybridScoring(2, 4, Button.kAutoButton, true).withTimeout(2));// score 2

        addCommands(chassis.followPPPath("2-LSL").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("LSL-4"));
        addCommands(new NewCoralHybridScoring(4, 4, Button.kAutoButton,true).withTimeout(2));// score 4

        addCommands(chassis.followPPPath("4-LSL").raceWith(new AutoIntakeCoral()));// intake

        addCommands(chassis.followPPPath("LSL-3"));
        addCommands(new NewCoralHybridScoring(3, 4, Button.kAutoButton,true).withTimeout(2));// score 3

        
    }
}
