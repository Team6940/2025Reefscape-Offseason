package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Library.MUtils;
import frc.robot.Library.team3476.net.editing.LiveEditableValue;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.commands.CoralCommands.ReversedCoralHybridScoring;
import frc.robot.commands.CoralCommands.NewReversedCoralHybridScoring;
import frc.robot.commands.CoralCommands.NewCoralHybridScoring;
import frc.robot.commands.GroundIntakeCommands.CoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.NewCoralAlignSequence;
// import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.AlgaeCommands.AlgaeHybridScoring;
import frc.robot.commands.Initialization;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperStructure extends SubsystemBase {

    private static SuperStructure m_Instance = null;

    public static SuperStructure getInstance() {
        return m_Instance == null ? m_Instance = new SuperStructure() : m_Instance;
    }

    public enum ScoreMode {
        PUSH,
        STOW
    }

    public enum Selection {
        LEFT,
        RIGHT
    }

    public enum RobotStatus {
        HOLDING_CORAL,
        HOLDING_ALGAE,
        IDLE
    }

    private RobotStatus robotStatus = RobotStatus.IDLE;
    private ScoreMode scoreMode = ScoreMode.STOW;
    Selection driverSelection = Selection.LEFT;



    /** Subsystems */
    private CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    private IntakerSubsystem intaker = IntakerSubsystem.getInstance();

    /** Variables */
    private int m_targetReefLevelIndex = 4;
    private int m_operatorReefFaceIndex = 3;
    // private int m_targetReefPoseIndex = 1;
    // private int m_targetStationPoseIndex = 1;
    // private int m_targetStationLevelIndex = 1; //TODO
    //private int m_targetAlgaeIntakeLevelIndex = 1;
    // private Field2d targetPoseField2d = new Field2d();
    // private Field2d targetStationPoseField2d = new Field2d();
    // private Field2d TEMP_stationCenterPose = new Field2d();
    private Field2d generatedReefPoseField2d = new Field2d();
    private Field2d generatedReefPoseReversedField2d = new Field2d();
    private Field2d generatedAlgaeScorePoseField2d = new Field2d();
    private Field2d generatedAlgaeIntakePoseField2d = new Field2d();



    public void setDriverSelection(Selection selection) {
        this.driverSelection = selection;
    }

    public void setControlState(ScoreMode newScoreMode) {
        scoreMode = newScoreMode;
    }

    public int getTargetReefLevelIndex() {
        return m_targetReefLevelIndex;
    }

    // public int getTargetReefPoseIndex() {
    //     return m_targetReefPoseIndex;
    // }

    // public int getTargetAlgaeIntakeLevelIndex() {
    //     return m_targetALgaeIntakeLevelIndex;
    // }

    public void setTargetReefLevelIndex(int index) {
        m_targetReefLevelIndex = index;
    }

    // public void setTargetAlgaeIntakeLevelIndex(int index) {
    //     m_targetALgaeIntakeLevelIndex = MUtils.numberLimit(0, 1, index); // TODO 0-1
    // }

    public void setOperatorReefFaceIndex(int index) {
        m_operatorReefFaceIndex = MUtils.numberLimit(1, 6, index);
    }

    // public void changeTargetAlgaeIntakeHeightIndex(int delta) {
    //     m_targetALgaeIntakeLevelIndex = MUtils.numberLimit(0, 1, delta); // TODO 0-1
    // }

    // public void changeTargetReefPoseIndex(int delta) {
    //     m_targetReefPoseIndex = MUtils.numberLimit(1, 12, delta); // TODO 1-12
    // }

    public void changeOperatorReefFaceIndex(Selection direction) { // TODO
        switch (direction) {
            case LEFT:
                switch (m_operatorReefFaceIndex) {
                    case 5:
                    case 6:
                        m_operatorReefFaceIndex = MUtils.cycleNumber(m_operatorReefFaceIndex, 1, 6, 1);
                        break;
                    case 1:
                    case 2:
                        break;
                    case 3:
                    case 4:
                        --m_operatorReefFaceIndex;
                        break;
                }
                break;
            case RIGHT:
                switch (m_operatorReefFaceIndex) {
                    case 1:
                    case 6:
                        m_operatorReefFaceIndex = MUtils.cycleNumber(m_operatorReefFaceIndex, 1, 6, -1);
                        break;
                    case 5:
                    case 4:
                        break;
                    case 2:
                    case 3:
                        ++m_operatorReefFaceIndex;
                        break;
                }
                break;
        }
    }

    public void changeTargetReefLevelIndex(int delta) {
        m_targetReefLevelIndex = MUtils.numberLimit(1, 4, m_targetReefLevelIndex + delta);
    }

    public Command getTraditionalHybridCoralScoreCommand(Button executionButton) {
        if (chassis.getToReefCenterDistance() <= FieldConstants.AutomaticallyAttachDistanceThreshold) {
            if (!chassis.isFacingReefCenter()) {
                return new ReversedCoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex,
                        executionButton).withSelection(driverSelection);
                // If the robot is not facing directly at the reef, it should do a reversed
                // score to avoid its arm hitting the reef
            } else {
                return new CoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, executionButton)
                        .withSelection(driverSelection);
            }
        } // If the bot is near the reef, it will automatically attach to the nearest reef
          // 'face'
        else {
            return new CoralHybridScoring(m_operatorReefFaceIndex * 2 - (driverSelection == Selection.LEFT ? 1 : 0),
                    m_targetReefLevelIndex, executionButton);
        } // Otherwise when the distance is too far, it will use the operator's selected
          // reef face
    }// This Command returns a CoralScore Command with push, so multiple factors
     // decide the way of method.

    public Command getNewHybridCoralScoreCommand(Button executionButton) {
        if (!chassis.isFacingReefCenter()) {
            return new NewCoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, executionButton, true)
                    .withSelection(driverSelection);
            // reversed scoring
        } else {
            return new NewCoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, executionButton,
                    false).withSelection(driverSelection);
            // non-reversed scoring
        }
    }
    // This command returns a CoralScore Command with no push since the bot is
    // already in STOW mode,
    // it should go directly to the scoring point despite its facing.

    public Command getHyrbidCoralScoreCommand(Button executionButton) {
        if (robotStatus == RobotStatus.HOLDING_CORAL) {
            if (scoreMode == ScoreMode.STOW) {
                return getNewHybridCoralScoreCommand(executionButton)
                        .andThen(() -> robotStatus = RobotStatus.IDLE);
            } else {
                return getTraditionalHybridCoralScoreCommand(executionButton)
                        .andThen(() -> robotStatus = RobotStatus.IDLE);
            }
        } else
            return null;
    }

    public Command getCoralAlignSequenceCommand(Button toggleButton) {
        if (robotStatus == RobotStatus.IDLE) {
            if (scoreMode == ScoreMode.STOW) {
                return new NewCoralAlignSequence(toggleButton).andThen(() -> robotStatus = RobotStatus.HOLDING_CORAL);
            } else {
                return new CoralAlignSequence(toggleButton).andThen(() -> robotStatus = RobotStatus.HOLDING_CORAL);
            }
        } else
            return null;
    }

    public Command getHybridAlgaeScoreCommand(Button triggeringButton, Button executionButton) {
        if (robotStatus == RobotStatus.HOLDING_ALGAE) {
            return new AlgaeHybridScoring(triggeringButton, executionButton)
                    .andThen(() -> robotStatus = RobotStatus.IDLE);
        } else
            return null;
    }

    public Command getHybridAlgaeIntakeCommand(Button executionButton) {

        if(robotStatus == RobotStatus.IDLE) {
            robotStatus = RobotStatus.HOLDING_ALGAE;
            return new AlgaeHybridIntake(chassis.generateAlgaeIntakeIndex(), executionButton).andThen(
                    () -> robotStatus = RobotStatus.HOLDING_ALGAE);
        }
        else return null;
    }

    public Command getInitializationCommand(Button toggleButton) {
        return new Initialization(toggleButton).andThen(() -> robotStatus = RobotStatus.IDLE);
    }

    public void switchScoreMode() {
        if (scoreMode == ScoreMode.PUSH) {
            scoreMode = ScoreMode.STOW;
        } else {
            scoreMode = ScoreMode.PUSH;
        }
    }

    @Override
    public void periodic() {
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states

        // SmartDashboard.putNumber("SuperStructure/operatorLevelIndex",m_operatorLevelIndex);

        // SmartDashboard.putData("SuperStructure/targetPoseField2d", targetPoseField2d);

        // generatedPoseField2d.setRobotPose(chassis.generateReefPose(chassis.generateReefIndex())); //TODO

        SmartDashboard.putNumber("SuperStructure/targetReefLevelIndex", m_targetReefLevelIndex);

        SmartDashboard.putNumber("SuperStructure/chassis.generateReefIndex()", chassis.generateReefIndex());

        SmartDashboard.putNumber("SuperStructure/generateAlgaeIntakeIndex()",chassis.generateAlgaeIntakeIndex());  // TODO

        SmartDashboard.putBoolean("SuperStructure/isFacingReefCenter()", chassis.isFacingReefCenter());

        // SmartDashboard.putData("SuperStructure/generatedPoseField2d", generatedPoseField2d); //TODO

        // targetStationPoseField2d.setRobotPose(chassis.generateStationPose());


        // SmartDashboard.putNumber("SuperStructure/targetStationPoseIndex", m_targetStationPoseIndex);
      
        // SmartDashboard.putData("SuperStructure/targetStationPoseField2d", targetStationPoseField2d);
      
        // TEMP_stationCenterPose.setRobotPose(chassis.generateAlignedStationPose());

        // TEMP_stationCenterPose.setRobotPose(chassis.generateStationPose());

        // SmartDashboard.putData("SuperStructure/TEMP_stationCenterPose", TEMP_stationCenterPose);

        // SmartDashboard.putBoolean("is STOW", scoreMode == ScoreMode.STOW);

        generatedReefPoseField2d.setRobotPose(chassis.generateReefPose(((chassis.generateReefIndex() - 1) / 2 * 2 + (driverSelection == Selection.LEFT ? 1 : 2))));

        generatedReefPoseReversedField2d.setRobotPose(chassis.generateReefPoseReversed(((chassis.generateReefIndex() - 1) / 2 * 2 + (driverSelection == Selection.LEFT ? 1 : 2))));
        
        generatedAlgaeIntakePoseField2d.setRobotPose(chassis.generateAlgaeIntakePose(chassis.generateAlgaeIntakeIndex()));

        generatedAlgaeScorePoseField2d.setRobotPose(chassis.generateAlgaeScorePose());

        // SmartDashboard.putData("SuperStructure/operatorPose(generatedReefPosField2d)", generatedReefPosField2d);

        SmartDashboard.putString("SuperStructure/RobotStatus", robotStatus.toString());

        SmartDashboard.putString("SuperStructure/driverSelection",driverSelection.toString());

        SmartDashboard.putString("SuperStructure/scoreMode", scoreMode.toString());

        // SmartDashboard.putString("SuperStructure/m_operatorReefFaceIndex(targetReefFace)",String.valueOf(m_operatorReefFaceIndex)); //TODO

        // SmartDashboard.putData("SuperStructure/generateAlgaeIntakePose()",); //TODO

        SmartDashboard.putData("SuperStructure/generateReefPose()",generatedReefPoseField2d); //TODO

        SmartDashboard.putData("SuperStructure/generateReefPoseReversed()",generatedReefPoseReversedField2d); //TODO

        SmartDashboard.putData("SuperStructure/generateAlgaeIntakePose()", generatedAlgaeIntakePoseField2d); //TODO

        SmartDashboard.putData("SuperStructure/generateAlgaeScorePose()", generatedAlgaeScorePoseField2d); //TODO
    }
}