package frc.robot.subsystems.Elevator;


public class ElevatorSubsystem {
        public static ElevatorSubsystem m_instance;
    public static ElevatorSubsystem getInstance() {
        return m_instance == null? m_instance = new ElevatorSubsystem() : m_instance;
    }
    
}
