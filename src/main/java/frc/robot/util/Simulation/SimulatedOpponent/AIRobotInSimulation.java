package frc.robot.util.Simulation.SimulatedOpponent;

import java.util.function.Supplier;

import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.TunerConstants;
import frc.robot.util.Simulation.MyDriveTrainSimulation;
import static edu.wpi.first.units.Units.*;

public class AIRobotInSimulation extends SubsystemBase {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double driveDeadband = 0.05;
    public static final double rotateDeadband = 0.09;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * driveDeadband).withRotationalDeadband(MaxAngularRate * rotateDeadband)
        .withDriveRequestType(DriveRequestType.Velocity);
        
    /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(-6, 0, new Rotation2d()),
        new Pose2d(-5, 0, new Rotation2d()),
        new Pose2d(-4, 0, new Rotation2d()),
        new Pose2d(-3, 0, new Rotation2d()),
        new Pose2d(-2, 0, new Rotation2d())
    };

    public final SwerveDriveSimulation driveSimulation;
    private final Pose2d queeningPose;
    private final int id;

    public AIRobotInSimulation(int id) {
        this.id = id;
        this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
        DriveTrainSimulationConfig DRIVETRAIN_CONFIG = MyDriveTrainSimulation._config;
        this.driveSimulation = new SwerveDriveSimulation(
            DRIVETRAIN_CONFIG, 
            queeningPose
        );

        SimulatedArena.getInstance().addDriveTrainSimulation(
            driveSimulation
        );
    }

    // PathPlanner configuration
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            55, // Robot mass in kg
            8,  // Robot MOI
            new ModuleConfig(
                    Units.inchesToMeters(2), 3.5, 1.2, DCMotor.getKrakenX60Foc(1).withReduction(8.14), 60, 1), // Swerve module config
            new Translation2d(0.6, 0.6) // Track length and width
    );

    // PathPlanner PID settings
    private final PPHolonomicDriveController driveController =
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.02), new PIDConstants(7.0, 0.05));

//     /** Follow path command for opponent robots */
//     private Command opponentRobotFollowPath(PathPlannerPath path) {
//                 path, // Specify the path
//                 // Provide actual robot pose in simulation, bypassing odometry error
//                 driveSimulation::getSimulatedDriveTrainPose,
//                 // Provide actual robot speed in simulation, bypassing encoder measurement error
//                 driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
//                 // Chassis speeds output
//                 (speeds, feedforwards) -> {
//                     // 将速度转换为 SwerveModuleState[]
//                     SwerveModuleState[] states = driveSimulation.kinematics.toSwerveModuleStates(speeds);
//                     // 调整轮速，确保不超过最大速度
//                     driveSimulation.kinematics.desaturateWheelSpeeds(states, driveSimulation.maxLinearVelocity().in(MetersPerSecond));

//                     // 更新每个模块的目标状态
//                     for (int i = 0; i < states.length; i++) {
//                         // 更新模块状态，计算出每个模块的力并应用
//                         driveSimulation.getModules()[i].updateSimulationSubTickGetModuleForce(
//                                 new Vector2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
//                                 states[i].angle, feedforwards.get(i).getMotorVoltage());
//                     }
//                 },
//                 driveController,
//                 PP_CONFIG, // Specify robot configuration
//                 // Flip path based on alliance side
//                 () -> DriverStation.getAlliance()
//                         .orElse(DriverStation.Alliance.Blue)
//                         .equals(DriverStation.Alliance.Red),
//                 this // AIRobotInSimulation is a subsystem; this command should use it as a requirement
//         );
//     }

    /** Build the behavior chooser of this opponent robot and send it to the dashboard */
    public void buildBehaviorChooser(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1,
            XboxController joystick) {
        SendableChooser<Command> behaviorChooser = new SendableChooser<>();
        final Supplier<Command> disable = () -> Commands.runOnce(() -> 
        driveSimulation.setSimulationWorldPose(queeningPose), this)
        .andThen(Commands.runOnce(() -> {
            // 将所有模块的速度设为0
            for (int i = 0; i < driveSimulation.getModules().length; i++) {
                SwerveModuleState zeroState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
                // 手动调用 updateSimulationSubTickGetModuleForce 来模拟每个模块的静止状态
                driveSimulation.getModules()[i].updateSimulationSubTickGetModuleForce(new Vector2(), zeroState.angle, 0.);
            }
        }))
        .ignoringDisable(true);

        // Option to disable the robot
        behaviorChooser.setDefaultOption("Disable", disable.get());

        // Option to auto-cycle the robot
        behaviorChooser.addOption(
                "Auto Cycle", getAutoCycleCommand(segment0, toRunAtEndOfSegment0, segment1, toRunAtEndOfSegment1));

        // Option to manually control the robot with a joystick
        behaviorChooser.addOption("Joystick Drive", joystickDrive(joystick));

        // Schedule the command when another behavior is selected
        behaviorChooser.onChange((Command::schedule));

        // Schedule the selected command when teleop starts
        RobotModeTriggers.teleop()
                .onTrue(Commands.runOnce(() -> behaviorChooser.getSelected().schedule()));

        // Disable the robot when the user robot is disabled
        RobotModeTriggers.disabled().onTrue(disable.get());

        SmartDashboard.putData("AIRobotBehaviors/Opponent Robot " + id + " Behavior", behaviorChooser);
    }

    private Command joystickDrive(XboxController driverController) { //TODO
        return Commands.run(() -> drive.withVelocityX(
                -driverController.getLeftY() 
                                        * Math.abs(driverController.getLeftY()) * MaxSpeed * 0.9)
                                .withVelocityY(
                                    -driverController.getLeftX() 
                                        * Math.abs(driverController.getLeftX()) * MaxSpeed * 0.9)
                                .withRotationalRate(
                                    -driverController.getRightX() * MaxAngularRate * 0.9));
    }

    /** Get the command to auto-cycle the robot relatively */
    private Command getAutoCycleCommand(
            PathPlannerPath segment0,
            Command toRunAtEndOfSegment0,
            PathPlannerPath segment1,
            Command toRunAtEndOfSegment1) {
        final SequentialCommandGroup cycle = new SequentialCommandGroup();
        final Pose2d startingPose = new Pose2d(
                segment0.getStartingDifferentialPose().getTranslation(),
                segment0.getIdealStartingState().rotation());

        // 添加路径跟随命令
        // cycle.addCommands(
        //         opponentRobotFollowPath(segment0).andThen(toRunAtEndOfSegment0).withTimeout(10));
        // cycle.addCommands(
        //         opponentRobotFollowPath(segment1).andThen(toRunAtEndOfSegment1).withTimeout(10));

        // 定义自动循环，反复执行路径
        return cycle.repeatedly()
                .beforeStarting(Commands.runOnce(() -> driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(startingPose))));
    }
}
