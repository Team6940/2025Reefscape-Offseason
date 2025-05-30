package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Library.MUtils;
import frc.robot.commands.Autos.*;

public class Robot extends TimedRobot { //TODO

  private Command m_autonomousCommand;
  public Command autoCommand = new Down3Corals();
  private int autoChoice = 3; 

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
      /*
       * (Ivan 2025.5.26):
       * 
       * This is for cycle selecting through 3 auto choices and select the autonomous command:
       * 1. Up3Corals
       * 2. Mid1Coral
       * 3. Down3Corals
       * 
       * (for more info look up in AutoGenerator.java)
       *  autoChoice variable : keep track of the current selection.
       * The autoCommand variable is updated with the new command based on the autoChoice.
      */
    if(RobotContainer.traditionalDriverController.getXButtonPressed()){
      autoChoice = MUtils.cycleNumber(autoChoice, 1, 3, -1);
      autoCommand = AutoGenerator.generate(autoChoice);
      /*
       * (Ivan 2025.5.26):
       * press X and autoChoice -1 
      */
    }
    if(RobotContainer.traditionalDriverController.getBButtonPressed()){
      autoChoice = MUtils.cycleNumber(autoChoice, 1, 3, 1);
      autoCommand = AutoGenerator.generate(autoChoice);
      /*
       * (Ivan 2025.5.26):
       * press B and autoChoice +1
      */
    }
    switch(autoChoice){

      //(Ivan 2025.5.26): Display autoChoice on your SmartDashboard

      case 1:
      SmartDashboard.putString("AutoType", "Up3Corals");
      break;
      case 2:
      SmartDashboard.putString("AutoType", "Mid1Coral");
      break;
      case 3:
      SmartDashboard.putString("AutoType", "Down3Corals");
      break;
    }
    
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

      // if(autoAlreadyRan = true) return;
      autoCommand.withTimeout(15.).schedule();
      // this is for the autonomous command to run only once
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    // (Ivan 2025.5.26): This method will be called once per scheduler run during simulation, not used in the real robot.
  }
}
