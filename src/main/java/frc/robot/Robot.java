package frc.robot;

import java.util.Arrays;
import java.util.List;
// import static edu.wpi.first.units.Units.Volts;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
// import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
// import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
// import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
// import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape.ReefscapeFieldObstacleMap;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.units.VoltageUnit;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos.*;
import frc.robot.constants.GeneralConstants;
import frc.robot.containers.RobotContainer;
import frc.robot.containers.SimContainer;
import frc.robot.library.MUtils;

public class Robot extends LoggedRobot { //TODO

  public static Object driveTrain;
  public Command autoCommand;
  private int autoChoice = 3; 

  // private RobotContainer m_robotContainer;
  // private TestContainer m_testContainer;
  private final RobotContainer m_robotContainer;
  private final SimContainer m_testContainer;

  public Robot() {
    GeneralConstants.initializeConstants();

    // Commands.either(
    //   Commands.runOnce(
    //     () -> m_robotContainer = new RobotContainer()),
    //   Commands.runOnce(
    //     () -> m_testContainer = new TestContainer()),
    //   RobotBase::isSimulation);

    this.m_robotContainer = new RobotContainer();
    this.m_testContainer = new SimContainer();
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    
    // // Obtains the default instance of the simulation world, which is a Crescendo Arena.
    // SimulatedArena.getInstance();
    // // Overrides the default simulation
    // SimulatedArena.overrideInstance(SimulatedArena.getInstance());
    /* THESE ARE COPIED FROM MAPLESIM.*/

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } 
    else {
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();
    /*THESE ARE COPIED FROM ADVANTAGEKIT.ORG DOCUMENTATION. */

  }

  @Override
  public void robotInit() {
        List<String> keys = Arrays.asList(
        "esc", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9", "f10", "f11", "f12",
        "delete", "`", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "backspace",
        "tab", "q", "w", "e", "r", "t", "y", "u", "i", "o", "p",
        "a", "s", "d", "f", "g", "h", "j", "k", "l", ";", "'",
        "shift", "z", "x", "c", "v", "b", "n", "m", ",", ".", "right shift",
        "ctrl", "alt", "right ctrl",
        "left", "right", "up", "down",
        "numpad0", "numpad1", "numpad2", "numpad3", "numpad4",
        "numpad5", "numpad6", "numpad7", "numpad8", "numpad9", "numpaddecimal"
    );

    for (String key : keys) {
        SmartDashboard.putBoolean("keyboard/" + key, false);
    }
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
      //  press povRight and autoChoice -1 
  
    }
    if(RobotContainer.traditionalDriverController.getPOV()==270){
      autoChoice = MUtils.cycleNumber(autoChoice, 1, 4, 1);
      autoCommand = AutoGenerator.generate(autoChoice);
      //  press povLeft and autoChoice +1
      
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
      autoCommand.withTimeout(15.).schedule();
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
    // SimulatedArena.getInstance().simulationPeriodic();
    // SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(1, 1, new Rotation2d(0.))));
    // SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));
    // SimulatedArena.getInstance().clearGamePieces();
    // // Get the positions of the notes (both on the field and in the air)
    //   Pose3d[] notesPoses = SimulatedArena.getInstance()
    //         .getGamePiecesArrayByType("Note");
    //   // Publish to telemetry using AdvantageKit
    //   Logger.recordOutput("FieldSimulation/NotesPositions", notesPoses);
    // // SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(3,3)));
    /* THESE ARE COPIED FROM MAPLESIM.*/

  // Logger.recordOutput("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
  // Logger.recordOutput("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
