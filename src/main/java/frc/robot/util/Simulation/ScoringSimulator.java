package frc.robot.util.Simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import frc.robot.library.ideasFrom6940.turretAiming.constants.TurretConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;

public class ScoringSimulator {
    public static void setCoralScoreSim(int levelIndex){
        Pose2d pose = CommandSwerveDrivetrain.getInstance().getPose();
        SimulatedArena.getInstance()
        .addGamePieceProjectile(new ReefscapeCoralOnFly(
            pose.getTranslation(),
            new Translation2d(0.35, 0), //TODO
            CommandSwerveDrivetrain.getInstance().getState().Speeds,
            pose.getRotation(),
            levelIndex == 1 ?
                Meters.of(0.76) : 
                levelIndex == 2 ? 
                    Meters.of(1.28) :
                    levelIndex == 3 ? 
                        Meters.of(1.84) :                   
                            Meters.of(1.97),
            MetersPerSecond.of(2), // The initial speed of the coral   
            Degrees.of(-35)) // The coral is ejected at a 35-degree slope
        .enableBecomesGamePieceOnFieldAfterTouchGround());
    }

    public static void setAlgaeScoreSim(){
        LinearVelocity launchVel = Units.MetersPerSecond.of(Robot.lastLaunchVel + 0.4); //TODO: I DONT KNOW WHY THE PREVIOUS VALUE JUST DOESNT WORK
        Angle elevationAng = Units.Degrees.of(Math.toDegrees(-Robot.lastPitch));

        Pose2d pose = CommandSwerveDrivetrain.getInstance().getPose();
        ReefscapeAlgaeOnFly.setHitNetCallBack(() -> System.out.println("ALGAE hits NET!"));
        SimulatedArena.getInstance()
            .addGamePieceProjectile(new ReefscapeAlgaeOnFly(
                pose.getTranslation(),
                new Translation2d(
                    TurretConstants.turretOffsetX,
                    TurretConstants.turretOffsetY),
                CommandSwerveDrivetrain.getInstance().getState().Speeds,
                new Rotation2d(
                    Robot.lastTurretYawX,
                    Robot.lastTurretYawY
                  ),
                Meters.of(TurretConstants.turretHeight),
                launchVel,
                elevationAng)
            .withProjectileTrajectoryDisplayCallBack(
                (poses) -> Logger.recordOutput("successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
                (poses) -> Logger.recordOutput("missedShotsTrajectory", poses.toArray(Pose3d[]::new)))
            .enableBecomesGamePieceOnFieldAfterTouchGround());
    }
}
