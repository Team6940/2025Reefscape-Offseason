package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public class ShooterIOInputs {
        public boolean leftMotorConnected;
        public boolean rghtMotorConnected;
        public double leftVoltageVolts;
        public double rghtVoltageVolts;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVelocityRPS;
        public double rghtVelocityRPS;
    }

    default public void updateInputs(ShooterIOInputs inputs) {}

    default public void setRPS(double rps) {}

    default public void setVoltage(double voltage) {}

    
}
