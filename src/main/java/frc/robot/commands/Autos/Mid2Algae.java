package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoPreparation;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.AlgaeCommands.AlgaeHybridScoring;
import frc.robot.commands.CoralCommands.NewCoralHybridScoring;
// import frc.robot.commands.CoralCommands.HybridScoring;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.commands.AlgaeCommands.AlgaeManualScoring;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Mid2Algae extends SequentialCommandGroup {
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();

    // ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public Mid2Algae() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            addCommands(new InstantCommand(
                    () -> chassis.resetPose(chassis.generatePPPath("MBM-11").getStartingHolonomicPose().get())));
        } else {
            addCommands(new InstantCommand(() -> chassis
                    .resetPose(chassis.generatePPPath("MBM-11").flipPath().getStartingHolonomicPose().get())));
        } // set original pose

        addCommands(chassis.followPPPath("MBM-11").alongWith(new AutoPreparation()));
        // addCommands(new CoralHybridScoring(11, 4, Button.kAutoButton));

        // addCommands(new AlgaeHybridIntake(6, Button.kAutoButton));w

        // addCommands(chassis.followPPPath("11-RBM"));
        // addCommands(new AlgaeHybridScoring(Button.kAutoButton, Button.kAutoButton));

        // addCommands(chassis.followPPPath("RBM-9-Algae"));
        // addCommands(new AlgaeHybridIntake(5, Button.kAutoButton));

        // addCommands(chassis.followPPPath("9-Algae-RBM"));
        // addCommands(new AlgaeHybridScoring(Button.kAutoButton, Button.kAutoButton));

    }

}
