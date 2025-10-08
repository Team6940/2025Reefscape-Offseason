package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.containers.RobotContainer;

public class Rumble extends Command{

    // (Ivan 2025.5.26): This makes the controller rumble.

    RumbleType m_RumbleType;
    double value=0.;
      public Rumble(RumbleType _RumbleType,double _value){
        
        m_RumbleType=_RumbleType;
        value=_value;
    }
    
    public void initialize()
    {
        RobotContainer.driverController.setRumble(m_RumbleType,value);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.driverController.setRumble(m_RumbleType,0.);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
