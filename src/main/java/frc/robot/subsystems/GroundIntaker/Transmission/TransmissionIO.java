package frc.robot.subsystems.GroundIntaker.Transmission;

import org.littletonrobotics.junction.AutoLog;

public interface TransmissionIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}

    @AutoLog
    public class ArmIOInputs {
        public boolean leftMotorConnected;
        public boolean rghtMotorConnected;

        public double leftVoltageVolts;
        public double rghtVoltageVolts;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVelocityRPS;
        public double rghtVelocityRPS;

    }

    @AutoLog
    public class ChannelIOInputs {
        public boolean leftMotorConnected;
        public boolean rghtMotorConnected;

        public double leftVoltageVolts;
        public double rghtVoltageVolts;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVelocityRPS;
        public double rghtVelocityRPS;

    }
    default public void updateInputs(ArmIOInputs inputs){}
}
