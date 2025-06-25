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

    public void setTargetReefLevelIndex(int index) {
        m_targetReefLevelIndex = index;
    }

    public void changeTargetReefLevelIndex(int delta) {
        m_targetReefLevelIndex = MUtils.numberLimit(1, 4, m_targetReefLevelIndex + delta);
    }

    public Command getHybridCoralCommand(Button button,Button resetButton,Button executionButton) {
        return new CoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, button, resetButton,executionButton).withSelection(driverSelection);
    }

    public Command getHybridAlgaeCommand(Button resetButton,Button executionButton) {
        return new AlgaeHybridScoring(chassis.generateReefIndex(), resetButton, executionButton);
    }


    public void periodic(){
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states
        // SmartDashboard.putNumber("SuperStructure/targetLevelIndex",); //TODO
    }

}
