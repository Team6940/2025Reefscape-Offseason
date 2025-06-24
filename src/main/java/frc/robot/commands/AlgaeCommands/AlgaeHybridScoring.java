// package frc.robot.commands.AlgaeCommands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.ShooterConstants;
// import frc.robot.subsystems.ImprovedCommandXboxController;
// import frc.robot.subsystems.Arm.ArmSubsystem;
// import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator.ElevatorSubsystem;
// import frc.robot.subsystems.ImprovedCommandXboxController.Button;
// import frc.robot.subsystems.Shooter.ShooterSubsystem;
// import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

// public class AlgaeHybridScoring extends Command {
//     enum ScoringState {
//         ALIGNING,
//         PUSHING,
//         SCORING,
//         DEPARTING,
//         END
//     }

//     private int m_targetReefPoseIndex;
//     private int m_targetReefLevelIndex;
//     private Button m_Button; //TODO: DESIGNED AS A TOGGLE BUTTON
//     private Button m_executionButton;
//     ScoringState state;

//     Pose2d targetPose;
//     double targetHeight;
//     double targetAngle;
//     double targetRotation;

//     ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
//     ShooterSubsystem shooter = ShooterSubsystem.getInstance();
//     ArmSubsystem arm = ArmSubsystem.getInstance();
//     CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
//     ImprovedCommandXboxController driverController = RobotContainer.driverController;

//     public AlgaeHybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button button,
//             Button executionButton) {
//         addRequirements(elevator, shooter, chassis, arm);
//         m_targetReefPoseIndex = targetReefPoseIndex;
//         m_targetReefLevelIndex = targetReefLevelIndex;
//         m_Button = button;
//         m_executionButton = executionButton;
//     }

//     @Override
//     public void initialize() {
//         state = ScoringState.ALIGNING;
//         targetPose = chassis.generateReefPose(m_targetReefPoseIndex);
//         targetHeight = FieldConstants.elevatorHeights[m_targetReefLevelIndex];
//         targetAngle = FieldConstants.armAngles[m_targetReefLevelIndex];
//         targetRotation = FieldConstants.reefRotationAdjustmentRange[m_targetReefPoseIndex];
//         elevator.setHeight(0);

//     }

//     @Override
//     public void execute() {
//         chassis.hybridMoveToPose(targetPose, driverController,
//                 FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
//         SmartDashboard.putString("ALGAE hybrid Scoring State", state.toString());
//         switch (state) {
//             case ALIGNING:
//                 align();
//                 break;
//             case SCORING:
//                 score();
//                 break;
//             case DEPARTING: //TODO
//                 depart();
//                 break;
//             case END:
//                 break;
//         }
//     }

//     public void align() {
//         SmartDashboard.putString("ALGAE hybrid Scoring State", "ALIGNING");
//         elevator.setHeight(targetHeight);
//         arm.setPosition(targetAngle);
//         if (arm.getArmPosition()> targetAngle - 0.1) {

//             state = ScoringState.SCORING;
//             SmartDashboard.putString("ALGAE hybrid Scoring State", "ALIGNING complete, moving to SCORING");
//         }
//     }

    // public void push() {
    //     SmartDashboard.putString("Hybrid Scoring State", "PUSHING");

    //     // Get current pose and add small forward offset (e.g. 0.2 meters)
    //     Pose2d currentPose = targetPose;
    //     Translation2d transformTranslation2d=new Translation2d(-FieldConstants.pushDistance, currentPose.getRotation());
    //     Pose2d pushPose=new Pose2d(currentPose.getTranslation().plus(transformTranslation2d), currentPose.getRotation());
    //     // Move to push position
    //     chassis.autoMoveToPose(pushPose);

    //     // When in position, transition to scoring
    //     if (chassis.isAtPose(pushPose) && driverController.getButton(m_executionButton)) {
    //         state = ScoringState.SCORING;
    //         SmartDashboard.putString("Hybrid Scoring State", "PUSHING complete, moving to SCORING");
    //     }
    // }

//     public void score() {
//         SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING");
//         arm.rotateArm(targetAngle);
//         if (shooter.getCoralState() == ShooterState.IDLE) {
//             SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING complete, moving to DEPARTING");
//             state = ScoringState.DEPARTING;
//         } else {
//             SmartDashboard.putString("ALGAE hybrid Scoring State", "SCORING in progress");
//         }

//     }

//     public void depart() {
//         SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING");

//         // Calculate retreat position (move backward)
//         Pose2d currentPose = targetPose;
//         Translation2d transformTranslation2d=new Translation2d(FieldConstants.pushDistance, currentPose.getRotation());
//         Pose2d departPose=new Pose2d(currentPose.getTranslation().plus(transformTranslation2d), currentPose.getRotation());

//         // Move to retreat position
//         targetPose=departPose;

//         shooter.setRPS(ShooterConstants.shooterScoringRPS);
//         // When in position, reset systems and end
//         if (chassis.isAtPose(departPose)) {
//             arm.setPosition(0);
//             elevator.setHeight(0);
//             shooter.stop();
//             state = ScoringState.END;
//             SmartDashboard.putString("ALGAE hybrid Scoring State", "DEPARTING complete, moving to END");
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if(interrupted)
//             SmartDashboard.putString("ALGAE hybrid Scoring State", "END due to interruption");
//         arm.stop();
//         elevator.setHeight(0);
//         shooter.stop();
//     }

// }
