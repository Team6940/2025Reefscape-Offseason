package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}

    @AutoLog
    public class IndexerIOInputs {
        public boolean IndexerMotorConnected;

        public double IndexerVoltageVolts;
        public double IndexerCurrentAmps;
        public double IndexerVelocityRPS;

    }
    default public void updateInputs(IndexerIOInputs inputs){}
}
