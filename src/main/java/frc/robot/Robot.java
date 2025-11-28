package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos.*;
import frc.robot.constants.GeneralConstants;
import frc.robot.containers.BaseContainer;
import frc.robot.containers.RobotContainer;
import frc.robot.containers.SimContainer;
import frc.robot.library.MUtils;
import frc.robot.library.ideasFrom6940.turretAiming.V2.AimCalculatorV2;
import frc.robot.library.ideasFrom6940.turretAiming.constants.NewFieldConstants;
import frc.robot.library.ideasFrom6940.turretAiming.constants.TurretConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller.KeyboardController;
import frc.robot.util.Simulation.GamePieceSimulator;

public class Robot extends LoggedRobot {

  private int autoChoice = 4; 
  public Command autoCommand = AutoGenerator.generate(autoChoice);
  private final BaseContainer m_baseContainer;
  private PowerDistribution m_powerDistribution;
  public final static Timer m_timer = new Timer();
  private int lastPOV = -1;
  private long lastChangeTime = 0;
  private static final int CYCLE_DELAY_MS = 300;
  public static double lastPitch =0.;
  private static double lastYaw =0.;
  public static double lastLaunchVel =0.;
  public static double lastTurretYawX =0.;
  public static double lastTurretYawY =0.;
  

  public Robot() {
    GeneralConstants.initializeConstants();
    // this.m_baseContainer = RobotBase.isSimulation() ? new SimContainer() : new RobotContainer();
    this.m_baseContainer = new RobotContainer();

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    this.m_powerDistribution = new PowerDistribution(2, ModuleType.kRev); // Enables power distribution logging

  }

  @Override
  public void robotInit() {
    Logger.start();
    m_timer.start();
    GamePieceSimulator.setGamePieceSim();

    Logger.recordOutput("AimingClac/Positions/Target", new Pose3d(
      NewFieldConstants.kTestScoringElement.getX(),
      NewFieldConstants.kTestScoringElement.getY(),
      NewFieldConstants.kTestScoringElement.getZ(),
      new Rotation3d()));    
    Logger.recordOutput("AimingClac/Positions/Turret", new Pose3d(
      0.,
      0.,
      0.,
      new Rotation3d()));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    KeyboardController.publishKeyboard();
    GamePieceSimulator.getGamePieces();

    Pose2d robotPose = CommandSwerveDrivetrain.getInstance().getPose();

    if(SimContainer.startAim){
      AimCalculatorV2.calculateOnce(
        robotPose.getX() + TurretConstants.turretOffsetX,
        robotPose.getY() + TurretConstants.turretOffsetY,
        TurretConstants.turretHeight,
        CommandSwerveDrivetrain.getInstance().getState().Speeds.vxMetersPerSecond,
        CommandSwerveDrivetrain.getInstance().getState().Speeds.vyMetersPerSecond,
        NewFieldConstants.kTestScoringElement.getX(),
        NewFieldConstants.kTestScoringElement.getY(),
        NewFieldConstants.kTestScoringElement.getZ());

      Logger.recordOutput("AimingClac/Positions/Turret", new Pose3d(
        robotPose.getX() + TurretConstants.turretOffsetX,
        robotPose.getY() + TurretConstants.turretOffsetY,
        TurretConstants.turretHeight,
        new Rotation3d(
          0.,
          -Math.toRadians(AimCalculatorV2.elevationAngle),
          Math.toRadians(AimCalculatorV2.azimuthAngle)
        )));
        lastPitch =-Math.toRadians(AimCalculatorV2.elevationAngle);
        lastYaw = Math.toRadians(AimCalculatorV2.azimuthAngle);
        lastTurretYawX = Math.cos(Math.toRadians(AimCalculatorV2.azimuthAngle));
        lastTurretYawY = Math.sin(Math.toRadians(AimCalculatorV2.azimuthAngle));
        lastLaunchVel = AimCalculatorV2.launchVelocity;

    }else {
      Logger.recordOutput("AimingClac/Positions/Turret", new Pose3d(
        robotPose.getX() + TurretConstants.turretOffsetX,
        robotPose.getY() + TurretConstants.turretOffsetY,
        TurretConstants.turretHeight,
        new Rotation3d(
          0.,
          lastPitch,
          lastYaw
        )
        ));
    }
    Logger.recordOutput("AimingClac/Status", AimCalculatorV2.status);
    Logger.recordOutput("AimingClac/LaunchVelocity", AimCalculatorV2.launchVelocity);
    Logger.recordOutput("AimingClac/ElevationAngle", AimCalculatorV2.elevationAngle);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    
      /*
       *  Cycle selecting through 4 auto choices and select the autonomous command:
       * 1. Up4Corals
       * 2. Mid1Coral
       * 3. Down4Corals
       * 4. LeftSide
      */
    int pov = m_baseContainer.getPOV();
    long now = System.currentTimeMillis();

    if (pov == 90 || pov == 270) {
        if (pov != lastPOV) {
            autoChoice = (pov == 90)
                ? MUtils.cycleNumber(autoChoice, 1, 4, -1)
                : MUtils.cycleNumber(autoChoice, 1, 4, 1);
            autoCommand = AutoGenerator.generate(autoChoice);
            lastChangeTime = now;
        } else if (now - lastChangeTime >= CYCLE_DELAY_MS) {
            autoChoice = (pov == 90)
                ? MUtils.cycleNumber(autoChoice, 1, 4, -1)
                : MUtils.cycleNumber(autoChoice, 1, 4, 1);
            autoCommand = AutoGenerator.generate(autoChoice);
            lastChangeTime = now;
        }
    }
    if (pov == 0) {
        autoChoice = 2;
        autoCommand = AutoGenerator.generate(autoChoice);
    }

    lastPOV = pov;

    switch(autoChoice){
      case 1: SmartDashboard.putString("AutoType", "Up4Corals");break;
      case 2: SmartDashboard.putString("AutoType", "Mid2Algae");break;
      case 3: SmartDashboard.putString("AutoType", "Down4Corals");break;
      case 4: SmartDashboard.putString("AutoType","LeftSide");break;
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
  public void testPeriodic() {
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
  }
}
