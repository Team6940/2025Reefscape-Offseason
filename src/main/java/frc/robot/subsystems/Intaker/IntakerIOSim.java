package frc.robot.subsystems.Intaker;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.util.Simulation.MyDriveTrainSimulation;

public class IntakerIOSim implements IntakerIO {
    private IntakeSimulation intakeSimulation;
    private AbstractDriveTrainSimulation driveTrainSimulation = new MyDriveTrainSimulation(MyDriveTrainSimulation._config, new Pose2d());
    private double intakeVoltage = 0.0;
    private double intakeRPS = 0.0;
    private boolean isIntakeRunningInSim = false;

    public IntakerIOSim() {
        this.driveTrainSimulation = new MyDriveTrainSimulation(MyDriveTrainSimulation._config,CommandSwerveDrivetrain.getInstance().getPose());
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveTrainSimulation,// Specify the drivetrain to which this intake is attached
            Inches.of(30.),
            Meters.of(2.),
            IntakeSimulation.IntakeSide.BACK,
            1
        );
        this.intakeSimulation.register();
    }

    /**
     * Check if there is coral in the intake
     * @return if there is coral in the intake
     */
    public boolean hasCoral() {
        return intakeSimulation.getGamePiecesAmount() != 0;
    }

    /**
     * Eject a coral from the intake (to be transferred to feeder/indexer/shooter)
     * @return if there is game piece(s) remaining, and therefore retrieved.
     */
    public boolean ejectCoral() {
        return intakeSimulation.obtainGamePieceFromIntake();
    }

    /**
     * Add a coral to the intake (to be called when a coral is intaken)
     * @return if the coral was successfully added.
     */
    public boolean addCoral() {
        return intakeSimulation.addGamePieceToIntake();
    }

    @Override // Defined by IntakeIO
    public void setVoltage(double voltage) {//TODO CHEAK PRACTICALITY: REAL
        this.intakeVoltage = voltage;
        if(voltage > 0.0) {
            intakeSimulation.startIntake();
            isIntakeRunningInSim = true;
        }
        else {
            intakeSimulation.stopIntake();
            isIntakeRunningInSim = voltage== 0. ? false : true;
        }
    }
    
    @Override // Defined by IntakeIO
    public void setRPS(double rps) {//TODO CHEAK PRACTICALITY: REAL 
        this.intakeRPS = rps;
        if(rps > 0.0) {
            intakeSimulation.startIntake();
            isIntakeRunningInSim = true;
        }
        else {
            intakeSimulation.stopIntake();
            isIntakeRunningInSim = rps == 0. ? false : true;
        }
    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.motorConnected = true; // Always connected in simulation
        inputs.motorVoltageVolts = intakeVoltage;
        inputs.motorCurrentAmps = 0.;
        inputs.intakerVelocityRPS = intakeRPS;
        inputs.hasGamePiece = hasCoral();
        inputs.gamepieces = intakeSimulation.getGamePiecesAmount();
        inputs.isIntakeRunning = isIntakeRunningInSim;
    }


}

