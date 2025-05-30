package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
// import frc.robot.subsystems.SuperStructure.Selection;

public class HybridScoring extends Command {
    // enum ScoringState {
    //     ALIGNING,
    //     AIMING,
    //     SCORING,
    //     END
    // };

    // private int m_targetReefPoseIndex;
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

    // public HybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button button, Button executionButton) {
    //     addRequirements(elevator, shooter, chassis);
    //     m_targetReefPoseIndex = targetReefPoseIndex;
    //     m_targetReefLevelIndex = targetReefLevelIndex;
    //     m_Button = button;
    //     m_executionButton = executionButton;
    // }

    // @Override
    // public void initialize() {
    //     state = ScoringState.ALIGNING;
    //     targetPose = chassis.generateReefPose(m_targetReefPoseIndex);
    //     targetHeight = FieldConstants.elevatorHeights[m_targetReefLevelIndex];
    //     // elevator.setHeight(FieldConstants.elevatorHeightsInAdvance[m_targetReefLevelIndex]);
    //     elevator.setHeight(0);
    // }

    // @Override
    // public void execute() {
    //     chassis.hybridMoveToPose(chassis.generateReefPose(m_targetReefPoseIndex), driverController, FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
    //     SmartDashboard.putBoolean("atTargetPose", chassis.isAtTargetPose());
    //     switch (state) {
    //         case ALIGNING:
    //             align();
    //             break;
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

    // private void align() {
    //     // double heightInAdvance=Math.min(FieldConstants.elevatorHeightInAdvanceMap.get(chassis.getToPoseDistance(targetPose)), targetHeight);
    //     if(chassis.getToPoseDistance(targetPose) < 0.45){
    //         elevator.setHeight(FieldConstants.elevatorHeightsInAdvance[m_targetReefLevelIndex]);
    //     }else if (chassis.getToPoseDistance(targetPose) > 0.5){
    //         elevator.setHeight(0);
    //     }
    //     if (chassis.isAtTargetPose()) {
    //         state = ScoringState.AIMING;
    //     }
    // }

    // private void aim() {
        
    //     // TODO: Probably need to push the bot forward a bit but that needs to be tested
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
    //     if(shooter.getCoralState() == CoralState.COMING_IN&&state==ScoringState.ALIGNING){
    //         return true;
    //     }
    //     if(!driverController.getButton(m_Button)) return true;
    //     if (state == ScoringState.END)
    //         return true;
    //     if(m_Button == Button.kAutoButton && shooter.getCoralState() == CoralState.OUT)
    //         return true;
    //     return false;
    // }

    // public HybridScoring withSelection(Selection selection){
    //     switch (selection) {
    //         case LEFT:
    //             return new HybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 1, m_targetReefLevelIndex, m_Button, m_executionButton);
    //         case RIGHT:
    //             return new HybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 2, m_targetReefLevelIndex, m_Button, m_executionButton);
    //         default:
    //             return null;
    //     }
    // }
};
