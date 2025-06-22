package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}
    default public void setLeftRPS(double rps){}
    default public void setRghtRPS(double rps){}

    @AutoLog
    public class IndexerIOInputs {
        public boolean leftMotorConnected;
        public boolean rghtMotorConnected;

        public double leftVoltageVolts;
        public double rghtVoltageVolts;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVelocityRPS;
        public double rghtVelocityRPS;


    }
    default public void updateInputs(IndexerIOInputs inputs){}
}
