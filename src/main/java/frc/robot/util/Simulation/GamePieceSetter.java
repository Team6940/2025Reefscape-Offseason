package frc.robot.util.Simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;

public class GamePieceSetter {

    public static void setGamePieceSim() {
        SimulatedArena.getInstance().clearGamePieces();

        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
            new Pose2d(7, 7, Rotation2d.fromDegrees(90))));
        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
            new Pose2d(4, 7,Rotation2d.fromDegrees(90))));

        SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(
            new Translation2d(5, 7)));

        SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(
            new Translation2d(6, 7)));
        
    }

    public static void clearGamePieces() {
        SimulatedArena.getInstance().clearGamePieces();
    }

    public static void setCoral_HP(String HPselection) {
        Commands.either(
            Commands.runOnce( ()->SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                new Pose2d(2, 4, Rotation2d.fromDegrees(0))))),
            Commands.runOnce( ()->SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
                new Pose2d(2, 4, Rotation2d.fromDegrees(0))))),
            () -> HPselection.equals("RIGHT"));
    }

     public static void getGamePieces() {
        Logger.recordOutput("FieldSimulation/Algae", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        Logger.recordOutput("FieldSimulation/Coral", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        }

     
}
