package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    private static SuperStructure m_Instance = null;

    public static SuperStructure getInstance() {
        return m_Instance == null ? m_Instance = new SuperStructure() : m_Instance;
    }

    /** Joysticks */
    private ImprovedCommandXboxController driverController;
    private ImprovedCommandXboxController operatorController;

        public SuperStructure() {
        // chassis = CommandSwerveDrivetrain.getInstance();
        // climber = ClimberSubsystem.getInstance();
        // elevator = ElevatorSubsystem.getInstance();
        // shooter = ShooterSubsystem.getInstance();
        driverController = new ImprovedCommandXboxController(0);
        operatorController = new ImprovedCommandXboxController(1);
    }

    public void periodic(){
        // This method will be called once per scheduler run
        // Add any periodic tasks here, such as updating telemetry or checking subsystem states
        //        SmartDashboard.putNumber("SuperStructure/targetLevelIndex",); //TODO
    }
}
