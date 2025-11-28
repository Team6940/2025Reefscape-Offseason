package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.AutoLog;

public interface IntakerIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}
    default public void updateInputs(IntakerIOInputs inputs){}
    @AutoLog
    public class IntakerIOInputs {
        public boolean motorConnected;
        public double motorVoltageVolts;
        public double motorCurrentAmps;
        public double intakerVelocityRPS;
        public boolean hasGamePiece;
        public int gamepieces;
        public boolean isIntakeRunning;
    }
    
}
