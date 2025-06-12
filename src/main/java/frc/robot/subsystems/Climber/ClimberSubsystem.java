package frc.robot.subsystems.Climber;
public class ClimberSubsystem {
    // This class is a placeholder for the Shooter subsystem.
    
    public static ClimberSubsystem m_instance;
    public static ClimberSubsystem getInstance() {
        return m_instance == null? m_instance = new ClimberSubsystem() : m_instance;
    }
    
    // Additional methods and properties can be added here as needed.
}
