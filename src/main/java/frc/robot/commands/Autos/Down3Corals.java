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

public class Down3Corals extends SequentialCommandGroup {
    //TODO: it's possible to use HybridCommands directly instead of FollowPath in order to save time
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    // ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    public Down3Corals() {
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("RBL-10").getStartingHolonomicPose().get())));
    //     }
    //     else{
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("RBL-10").flipPath().getStartingHolonomicPose().get())));
    //     }
    //     addCommands(chassis.followPPPath("RBL-10").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(10, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("10-RSM"));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));

    //     addCommands(chassis.followPPPath("RSM-8").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(8, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("8-RSM"));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));
        
    //     addCommands(chassis.followPPPath("RSM-7").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(7, 4, Button.kAutoButton, Button.kAutoButton));

    //     addCommands(chassis.followPPPath("7-RSM").alongWith(new AlgaeRemovalCommand(Button.kA).withTimeout(1)));
    //     addCommands(Commands.defer(() -> new HybridCoralIn(Button.kAutoButton), Set.of(elevator, shooter, chassis)));

    //     addCommands(chassis.followPPPath("RSM-7").raceWith(new CentreCoralPos()));
    //     addCommands(new HybridScoring(7, 3, Button.kAutoButton, Button.kAutoButton));
    // }

    // public Pose2d getStartingPose(){
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         return chassis.generatePPPath("RBL-10").getStartingHolonomicPose().get();
    //     }
    //     else{
    //         return chassis.generatePPPath("RBL-10").flipPath().getStartingHolonomicPose().get();
    //     }
    }
}
