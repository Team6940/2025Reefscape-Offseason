package frc.robot.util.Simulation.SimulatedOpponent;
// package frc.robot.subsystems.Simulation.SimulateOpponent;

// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AIRobotInSimulation extends SubsystemBase {
//     /* If an opponent robot is not on the field, it is placed in a queening position for performance. */
//     public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
//         new Pose2d(-6, 0, new Rotation2d()),
//         new Pose2d(-5, 0, new Rotation2d()),
//         new Pose2d(-4, 0, new Rotation2d()),
//         new Pose2d(-3, 0, new Rotation2d()),
//         new Pose2d(-2, 0, new Rotation2d())
//     };

//     private final SwerveDriveSimulation driveSimulation;
//     private final Pose2d queeningPose;
//     private final int id;

//     public AIRobotInSimulation(int id) {
//         this.id = id;
//         this.queeningPose = ROBOT_QUEENING_POSITIONS[id];
//         DriveTrainSimulationConfig DRIVETRAIN_CONFIG = null;
//         this.driveSimulation = new SwerveDriveSimulation(
//             DRIVETRAIN_CONFIG, 
//             queeningPose
//         );

//         SimulatedArena.getInstance().addDriveTrainSimulation(
//             driveSimulation
//         );
//     }
// }