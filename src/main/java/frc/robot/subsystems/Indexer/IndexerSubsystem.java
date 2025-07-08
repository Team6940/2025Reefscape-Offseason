package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    public static IndexerSubsystem m_instance;

    public static IndexerSubsystem getInstance() {
        return m_instance == null ? m_instance = new IndexerSubsystem() : m_instance;
    }


    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged(); // TODO


    private double targetRPS = 0.;
    public IndexerState state;

    public enum IndexerState {
        IDLE,
        // ALIGNING,
        // FREE_SPINNING,
        READY
    }

    public IndexerState getIndexerState() {
        return state;
    }

    public IndexerSubsystem() {
        if (Robot.isReal()) {
            io = new IndexerIOPhoenix6();
        } else {
            // TODO: Implement simulation code here
            io = new IndexerIOPhoenix6();
        }
    }

    public void setRPS(double rps) {
        targetRPS = rps;
        io.setRPS(rps);
    }

    public void setLeftRPS(double rps) {
        io.setLeftRPS(rps);
    }

    public void setRghtRPS(double rps) {
        io.setRghtRPS(rps);
    }

    public boolean IsAtTargetRPS() {
        return MathUtil.isNear(targetRPS, (inputs.leftVelocityRPS + inputs.rghtVelocityRPS) / 2.,
                IndexerConstants.IndexerVelocityToleranceRPS);
    }

    public void stop() {
        setRPS(0.);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void indexerStateUpdate(){      
        if (inputs.sensorGet) {
            state = IndexerState.READY;
        } else{
            state = IndexerState.IDLE;
        }
        // double avgCurrent= (inputs.leftCurrentAmps + inputs.rghtCurrentAmps) / 2.0;
        // if (avgCurrent < IndexerConstants.IndexerFreeSpinCurrentThreshold) {
        //     state = IndexerState.IDLE;
        // } else if (avgCurrent < IndexerConstants.IndexerAligningCurrentThreshold) {
        //     state = IndexerState.FREE_SPINNING;
        // } else {
        //     state = IndexerState.ALIGNING;
        // }
    }

    @Override
    public void periodic() {
        indexerStateUpdate();
        processLog();
        processDashboard();
    }

    private void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("Indexer/TargetRPS", targetRPS);
        Logger.recordOutput("Indexer/IsAtTargetRPS", IsAtTargetRPS());
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here
    }

}
