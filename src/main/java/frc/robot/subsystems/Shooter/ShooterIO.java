package frc.robot.subsystems.Shooter;

public interface ShooterIO {

    default public void setVoltage(double voltage) {}

    default public void setRPS(double rps) {}

    // @org.littletonrobotics.junction.AutoLog
    public class ShooterIOInputs {
        public boolean leftMotorConnected;
        public boolean rightMotorConnected;

        public double leftVoltageVolts;
        public double rightVoltageVolts;
        public double leftCurrentAmps;
        public double rightCurrentAmps;
        public double leftVelocityRPS;
        public double rightVelocityRPS;
    }

    default public void updateInputs(ShooterIOInputs inputs) {}
}