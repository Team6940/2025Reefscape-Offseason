package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
        public static ElevatorSubsystem m_instance;
    public static ElevatorSubsystem getInstance() {
        return m_instance == null? m_instance = new ElevatorSubsystem() : m_instance;
    }
    
}
