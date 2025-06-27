package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Library.MUtils;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.commands.*;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake;
import frc.robot.commands.AlgaeCommands.AlgaeHybridScoring;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperStructure extends SubsystemBase{

    private static SuperStructure m_Instance = null;
    public static SuperStructure getInstance() {
        return m_Instance == null ? m_Instance = new SuperStructure() : m_Instance;
    }

    public enum ControlState {
        HYBRID,
        MANUAL
    }
    public enum Selection {
        LEFT,
        RIGHT
    }

    private ControlState controlState = ControlState.HYBRID;
    Selection driverSelection = Selection.RIGHT;

    /** Subsystems */
    private CommandSwerveDrivetrain chassis;
    private ArmSubsystem groundIntaker; 
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private ClimberSubsystem climber; 
    

    /** Joysticks */
    private ImprovedCommandXboxController driverController;
    private ImprovedCommandXboxController operatorController;

    /** Variables */ //TODO
    private int m_targetReefPoseIndex = 1;
    private int m_targetReefLevelIndex = 4;
    private int m_targetStationPoseIndex = 1;
    private int m_operatorReefFaceIndex = 3;
    private int m_targetStationLevelIndex = 1; //TODO
    private int m_targetALgaeIntakeLevelIndex = 1; //TODO

    public SuperStructure() {
        chassis = CommandSwerveDrivetrain.getInstance();
        groundIntaker = ArmSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();
        elevator = ElevatorSubsystem.getInstance();
        climber = ClimberSubsystem.getInstance();
        
        driverController = new ImprovedCommandXboxController(0);
        operatorController = new ImprovedCommandXboxController(1);

    }

    public void setControlState(ControlState newState){
        controlState = newState;
    }

    public int getTargetReefLevelIndex() {
        return m_targetReefLevelIndex;
    }

    public int getTargetReefPoseIndex() {
        return m_targetReefPoseIndex;
    }

    public int getTargetAlgaeIntakeLevelIndex() {
        return m_targetALgaeIntakeLevelIndex;
    }

    public void setTargetReefLevelIndex(int index) {
        m_targetReefLevelIndex = index;
    }

    public void setTargetAlgaeIntakeLevelIndex(int index) {
        m_targetALgaeIntakeLevelIndex = MUtils.numberLimit(0, 1, index); //TODO 0-1
    }

    public void setOperatorReefFaceIndex(int index){
        m_operatorReefFaceIndex = MUtils.numberLimit(1, 6, index);
    }

    public void changeTargetAlgaeIntakeHeightIndex(int delta) {
        m_targetALgaeIntakeLevelIndex = MUtils.numberLimit(0, 1, delta); //TODO 0-1
    }


    public void changeTargetReefPoseIndex(int delta) {
        m_targetReefPoseIndex = MUtils.numberLimit(1, 12, delta); //TODO 1-12
    }

    public void changeOperatorReefFaceIndex(Selection direction){ //TODO
        switch (direction) {
            case LEFT:
                switch(m_operatorReefFaceIndex){
                    case 5: case 6:
                        m_operatorReefFaceIndex = MUtils.cycleNumber(m_operatorReefFaceIndex, 1, 6, 1);
                        break;
                    case 1: case 2:
                        break;
                    case 3: case 4:
                        --m_operatorReefFaceIndex;
                        break;
                }
                break;
            case RIGHT:
                switch(m_operatorReefFaceIndex){
                    case 1: case 6:
                        m_operatorReefFaceIndex = MUtils.cycleNumber(m_operatorReefFaceIndex, 1, 6, -1);
                        break;
                    case 5: case 4:
                        break;
                    case 2: case 3:
                        ++m_operatorReefFaceIndex;
                        break;
                }
                break;
        }
    }

    public void changeTargetReefLevelIndex(int delta) {
        m_targetReefLevelIndex = MUtils.numberLimit(1, 4, m_targetReefLevelIndex + delta);
    }

    public Command getHybridCoralCommand(Button locateButton,Button executionButton) {
        return new CoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, locateButton,executionButton).withSelection(driverSelection);
    }

    public Command getHybridAlgaeCommand(Button resetButton,Button executionButton) {
        return new AlgaeHybridScoring(executionButton);
    }

    public Command getHybridAlgaeIntakeCommand(Button executionButton) {
        return new AlgaeHybridIntake(m_targetReefPoseIndex, m_targetALgaeIntakeLevelIndex, executionButton); //TODO
    }


    public void periodic(){
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states
        // SmartDashboard.putNumber("SuperStructure/targetLevelIndex",); //TODO
    }

}
