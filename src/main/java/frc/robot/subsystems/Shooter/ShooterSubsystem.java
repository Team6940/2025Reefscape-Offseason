package frc.robot.subsystems.Shooter;

import frc.robot.subsystems.GroundIntaker.G_Arm.G_ArmSubsystem;

public class ShooterSubsystem {
    // This class is a placeholder for the Shooter subsystem.
    // It can be expanded with methods and properties related to the Shooter functionality.
    
    public static ShooterSubsystem m_instance;
    public static ShooterSubsystem getInstance() {
        return m_instance == null? m_instance = new ShooterSubsystem() : m_instance;
    }

    public void setVoltage(double voltage) {
        // Set the voltage for the shooter motors
    }

    public void setRPS(double rps) {
        // Set the revolutions per second for the shooter motors
    }

    // Additional methods and properties can be added here as needed.
}
