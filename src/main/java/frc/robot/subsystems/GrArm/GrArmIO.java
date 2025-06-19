package frc.robot.subsystems.GrArm;

import org.littletonrobotics.junction.AutoLog;

public interface GrArmIO {

    @AutoLog
    public class GrArmIOInputs {
        public boolean motorConnected;
        public double motorVoltageVolts;
        public double motorCurrentAmps;
        public double GrArmVelocityRPS;
    }

    default public void updateInputs(GrArmIOInputs inputs) {}

    default public void setRPS(double rps) {}

    default public void setVoltage(double voltage) {}
    
}