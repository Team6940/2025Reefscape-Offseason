package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.AutoLog;

public interface IntakerIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}

    @AutoLog
    public class IntakerIOInputs {
        public boolean leftMotorConnected;
        public boolean rghtMotorConnected;

        public double leftVoltageVolts;
        public double rghtVoltageVolts;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVelocityRPS;
        public double rghtVelocityRPS;

    }
    default public void updateInputs(IntakerIOInputs inputs){}
}
