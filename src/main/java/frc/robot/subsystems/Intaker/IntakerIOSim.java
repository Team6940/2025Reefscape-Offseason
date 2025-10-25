package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Simulation.MyDriveTrainSimulation;

public class IntakerIOSim implements IntakerIO {
    private IntakeSimulation intakeSimulation;
    private AbstractDriveTrainSimulation driveTrainSimulation = new MyDriveTrainSimulation(MyDriveTrainSimulation._config, new Pose2d());
    private double intakeVoltage = 0.0;
    private double intakeRPS = 0.0;

    public IntakerIOSim() {
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveTrainSimulation,// Specify the drivetrain to which this intake is attached
            Inches.of(28.), //TODO The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2), //TODO The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            1
        );
        this.intakeSimulation.register();
    }
    /** Simulation control interface for Subsystem calls.
     * 
     * @param run
    */
    public void setRunningState(boolean run) {
        if (run) intakeSimulation.startIntake();
        else intakeSimulation.stopIntake();
    }

    public boolean hasCoral() {
        return intakeSimulation.getGamePiecesAmount() > 0;
    }

    /**
     * Eject a coral from the intake (to be transferred to feeder/indexer/shooter)
     * @return if there is game piece(s) remaining, and therefore retrieved.
     */
    public boolean ejectCoral() {
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    public boolean addCoral() {
        return intakeSimulation.addGamePieceToIntake();
    }

    @Override // Defined by IntakeIO
    public void setVoltage(double voltage) {
        this.intakeVoltage = voltage;
    }
    
    @Override // Defined by IntakeIO
    public void setRPS(double rps) {
        this.intakeRPS = rps;
    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        intakeSimulation.removeObtainedGamePieces(SimulatedArena.getInstance());

        inputs.motorConnected = true; // Always connected in simulation
        inputs.motorVoltageVolts = intakeVoltage;
        inputs.motorCurrentAmps = 3.; //TODO
        inputs.intakerVelocityRPS = intakeRPS;
        inputs.hasGamePiece = hasCoral();
    }


}

