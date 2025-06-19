package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public class ShooterIOInputs {
        public boolean motorConnected;
        public double motorVoltageVolts;
        public double motorCurrentAmps;
        public double shooterVelocityRPS;
    }

    default public void updateInputs(ShooterIOInputs inputs) {}

    default public void setRPS(double rps) {}

    default public void setVoltage(double voltage) {}

    
}
