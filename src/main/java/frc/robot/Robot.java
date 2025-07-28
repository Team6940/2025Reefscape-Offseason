package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Library.MUtils;
import frc.robot.commands.Autos.*;

public class Robot extends LoggedRobot { //TODO

  private Command m_autonomousCommand;
  public Command autoCommand = new Down4Corals();
  private int autoChoice = 3; 

  private final RobotContainer m_robotContainer;

  public Robot() {
    Constants.initializeConstants();
    m_robotContainer = new RobotContainer();
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    //TODO: copied from 2025Reefscape

    ///
    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } 
    else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();
    ///This is copied from advantagekit.org documentation.

  }

  @Override
  public void robotInit() {
    // CameraServer.startAutomaticCapture();
    // CameraServer.putVideo("limelight_front", 640, 480);
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
       * for cycle selecting through 3 auto choices and select the autonomous command:
       * 1. Up4Corals
       * 2. Mid1Coral
       * 3. Down4Corals
       * 
       * (for more info look up in AutoGenerator.java)
       * autoChoice : keep track of the current selection.
       * The autoCommand variable is updated with the new command based on the autoChoice.
      */
    if(RobotContainer.traditionalDriverController.getPOV() == 90){
      autoChoice = MUtils.cycleNumber(autoChoice, 1, 4, -1);
      autoCommand = AutoGenerator.generate(autoChoice);
      /*
       * press povRight and autoChoice -1 
      */
    }
    if(RobotContainer.traditionalDriverController.getPOV()==270){
      autoChoice = MUtils.cycleNumber(autoChoice, 1, 4, 1);
      autoCommand = AutoGenerator.generate(autoChoice);
      /*
       * press povLeft and autoChoice +1
      */
    if(RobotContainer.traditionalDriverController.getPOV()==0){
      autoChoice=2;
      autoCommand=AutoGenerator.generate(autoChoice);
    }
    }
    switch(autoChoice){

      //Display autoChoice on your SmartDashboard

      case 1:
        SmartDashboard.putString("AutoType", "Up4Corals");
        break;
      case 2:
        SmartDashboard.putString("AutoType", "Mid2Algae");
        break;
      case 3:
        SmartDashboard.putString("AutoType", "Down4Corals");
        break;
      case 4:
        SmartDashboard.putString("AutoType","LeftSide");
        break;
    }
    
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
      // if(autoAlreadyRan = true) return;
      autoCommand.withTimeout(15.).schedule();
      // RobotContainer.chassis.followPPPath("1").withTimeout(3).schedule();
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
    //This method will be called once per scheduler run during simulation, not used in the real robot.
  }
}
