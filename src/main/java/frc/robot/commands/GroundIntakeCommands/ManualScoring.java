package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem.CoralState;
import frc.robot.Constants.ManualConstants;

public class ManualScoring extends Command {
    // enum ScoringState {
    //     AIMING,
    //     SCORING,
    //     END
    // };

    // private int m_targetReefLevelIndex;
    // private Button m_Button;
    // private Button m_executionButton;
    // ScoringState state;

    // Pose2d targetPose;
    // double targetHeight;

    // ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    // CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    // ImprovedCommandXboxController driverController = RobotContainer.driverController;

    // public ManualScoring(ImprovedCommandXboxController controller,int targetReefLevelIndex, Button button, Button executionButton) {
    //     addRequirements(elevator, shooter, chassis);
    //     m_Button = button;
    //     m_executionButton = executionButton;
    //     m_targetReefLevelIndex = targetReefLevelIndex;
    // }

    // @Override
    // public void initialize() {
    //     state = ScoringState.AIMING;
    //     targetHeight = FieldConstants.elevatorHeights[m_targetReefLevelIndex];
    //     elevator.setHeight(FieldConstants.elevatorHeights[m_targetReefLevelIndex]);
    // }

    // @Override
    // public void execute() {
    //     chassis.driveFieldCentric(driverController, 1, ManualConstants.maxSpeed);
    //     switch (state) {
    //         case AIMING:
    //             aim(); 
    //             break;
    //         case SCORING:
    //             score();
    //             break;
    //         case END:
    //             break;
    //     }
    // }

    // private void aim() {
    //     elevator.setHeight(targetHeight);
    //     if (elevator.isAtTargetHeight() && driverController.getButton(m_executionButton)) {
    //         state = ScoringState.SCORING;
    //     }
    // }

    // private void score() {
    //     // shooter.setRPS(ShooterConstants.ShooterShootRPS);
    //     shooter.setRPS(shooter.getShootRPS(m_targetReefLevelIndex));
    //     if (!driverController.getButton(m_executionButton)) {
    //         state = ScoringState.END;
    //     }
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     elevator.setHeight(0);
    //     shooter.stop();
    // }

    // @Override
    // public boolean isFinished() {
    //     if(shooter.getCoralState() == CoralState.COMING_IN&&state==ScoringState.AIMING){
    //         return true;
    //     }
    //     if(!driverController.getButton(m_Button)) return true;
    //     if (state == ScoringState.END)
    //         return true;
    //     return false;
    // }
};
