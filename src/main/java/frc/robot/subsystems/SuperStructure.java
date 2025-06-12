package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.commands.*;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperStructure extends SubsystemBase{

    private static SuperStructure m_Instance = null;
    public static SuperStructure getInstance() {
        return m_Instance == null ? m_Instance = new SuperStructure() : m_Instance;
    }

    public enum ExampleState { //TODO
        STATE_ONE,
        STATE_TWO,
        STATE_THREE
    }

    /** Subsystems */
    private CommandSwerveDrivetrain chassis;
    private ArmSubsystem groundIntaker; 
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private ClimberSubsystem climber; 
    

    /** Joysticks */
    private ImprovedCommandXboxController driverController;

    /** Variables */
    //TODO

    public SuperStructure() {
        chassis = CommandSwerveDrivetrain.getInstance();
        groundIntaker = ArmSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();
        elevator = ElevatorSubsystem.getInstance();
        climber = ClimberSubsystem.getInstance();
        
        driverController = new ImprovedCommandXboxController(0);
    }

    //TODO:add requirements here

    public void periodic(){
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states
        // SmartDashboard.putNumber("SuperStructure/targetLevelIndex",); //TODO
    }

}
