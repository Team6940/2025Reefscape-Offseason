package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.CoralCommands.HybridScoring;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Mid1Coral extends SequentialCommandGroup{
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    // ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    public Mid1Coral(){
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("MBM-11").getStartingHolonomicPose().get())));
    //     }
    //     else{
    //         addCommands(new InstantCommand(() -> chassis.resetPose(chassis.generatePPPath("MBM-11").flipPath().getStartingHolonomicPose().get())));
    //     }
    //     addCommands(chassis.followPPPath("MBM-11"));
    //     addCommands(new HybridScoring(11, 4, Button.kAutoButton, Button.kAutoButton));
    // }
    // public Pose2d getStartingPose(){
    //     if(DriverStation.getAlliance().get() == Alliance.Blue){
    //         return chassis.generatePPPath("MBM-11").getStartingHolonomicPose().get();
    //     }
    //     else{
    //         return chassis.generatePPPath("MBM-11").flipPath().getStartingHolonomicPose().get();
    //     }
    }

}
