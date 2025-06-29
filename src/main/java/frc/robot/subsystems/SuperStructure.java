package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.commands.CoralCommands.CoralHybridScoring;
import frc.robot.commands.GroundIntakeCommands.CoralAlignSequence;
import frc.robot.commands.GroundIntakeCommands.ToggleIntake;
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
    private ArmSubsystem Arm; 
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private ClimberSubsystem climber; 
    private GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    private IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    

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
    private Field2d targetPoseField2d = new Field2d();
    private Field2d generatedPoseField2d = new Field2d();
    private Field2d targetStationPoseField2d = new Field2d();
    private Field2d TEMP_stationCenterPose = new Field2d();
    private Field2d operatorTargetPoseField2d = new Field2d();

    public SuperStructure() {
        chassis = CommandSwerveDrivetrain.getInstance();
        Arm= ArmSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();
        elevator = ElevatorSubsystem.getInstance();
        climber = ClimberSubsystem.getInstance();
        grArm = GrArmSubsystem.getInstance();
        intaker = IntakerSubsystem.getInstance();
        driverController = new ImprovedCommandXboxController(0);
        operatorController = new ImprovedCommandXboxController(1);

    }

    public void setDriverSelection(Selection selection){
        this.driverSelection = selection;
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

    public Command getHybridCoralScoreCommand(Button executionButton) {
        if(chassis.getToReefCenterDistance()<=FieldConstants.AutoMaticllyAttachDistanceThreshold){
            return new CoralHybridScoring(chassis.generateReefIndex(), m_targetReefLevelIndex, executionButton).withSelection(driverSelection);
        }//If the bot is near the reef, it will automatically attach to the nearest reef 'face'
        else {
            return new CoralHybridScoring(m_operatorReefFaceIndex * 2 - (driverSelection == Selection.LEFT ? 1 : 0), m_targetReefLevelIndex, executionButton);
        }//Otherwise when the distance is too far, it will use the operator's selected reef face
    }

    public Command getHybridAlgaeScoreCommand(Button executionButton) {
        return new AlgaeHybridScoring(executionButton);
    }

    public Command getHybridAlgaeIntakeCommand(Button executionButton) {
        return new AlgaeHybridIntake(chassis.generateAlgaeIntakeIndex(), m_targetALgaeIntakeLevelIndex, executionButton); //TODO
    }

    public Command getCoralAlignSequenceCommand() {
        if(intaker.isReady()){
            return new CoralAlignSequence();
        }
        else return null;
    }


    public void periodic(){
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states
        SmartDashboard.putNumber("SuperStructure/targetLevelIndex", m_targetReefLevelIndex);
        // SmartDashboard.putNumber("SuperStructure/operatorLevelIndex", m_operatorLevelIndex);

        SmartDashboard.putData("SuperStructure/targetPose", targetPoseField2d);

        generatedPoseField2d.setRobotPose(chassis.generateReefPose(chassis.generateReefIndex()));
        SmartDashboard.putNumber("SuperStructure/AutoGeneratedTargetPoseIndex", chassis.generateReefIndex());
        SmartDashboard.putData("SuperStructure/AutoGeneratedTargetPose", generatedPoseField2d);

        targetStationPoseField2d.setRobotPose(chassis.generateStationPose());
        SmartDashboard.putNumber("SuperStructure/targetStationPoseIndex", m_targetStationPoseIndex);
        SmartDashboard.putData("SuperStructure/targetStationPose", targetStationPoseField2d);

        TEMP_stationCenterPose.setRobotPose(chassis.generateAlignedStationPose());
        // TEMP_stationCenterPose.setRobotPose(chassis.generateStationPose());
        SmartDashboard.putData("SuperStructure/TEMPStationPose", TEMP_stationCenterPose);

        SmartDashboard.putBoolean("isHybrid", controlState == ControlState.HYBRID);

        operatorTargetPoseField2d.setRobotPose(chassis.generateReefPose(m_operatorReefFaceIndex * 2 - (driverSelection == Selection.LEFT ? 1 : 0)));
        SmartDashboard.putData("SuperStructure/operatorPose", operatorTargetPoseField2d);    
    }
}