package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AlgaeRemovalCommand;
// import frc.robot.commands.CoralCommands.HybridScoring;
// import frc.robot.commands.IntakeCommands.CentreCoralPos;
// import frc.robot.commands.IntakeCommands.HybridCoralIn;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;

public class Up3Corals extends SequentialCommandGroup {
    //TODO: it's possible to use HybridCommands directly instead of FollowPath in order to save time
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    // ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Up3Corals() {
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("LBR-1").getStartingHolonomicPose().get())));
    //     }
    //     else{
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("LBR-1").flipPath().getStartingHolonomicPose().get())));
    //     }
    //     addCommands(chassis.followPPPath("LBR-1").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(1, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("1-LSM"));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));

    //     addCommands(chassis.followPPPath("LSM-3").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(3, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("3-LSM"));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));
        
    //     addCommands(chassis.followPPPath("LSM-4").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(4, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("4-LSM").alongWith(new AlgaeRemovalCommand(Button.kA).withTimeout(1.)));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));

    //     addCommands(chassis.followPPPath("LSM-4").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(4, 3, Button.kAutoButton, Button.kAutoButton));
    // }

    // public Pose2d getStartingPose(){
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         return chassis.generatePPPath("LBR-1").getStartingHolonomicPose().get();
    //     }
    //     else{
    //         return chassis.generatePPPath("LBR-1").flipPath().getStartingHolonomicPose().get();
    //     }
    }
}
