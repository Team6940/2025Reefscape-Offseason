package frc.robot.subsystems.Shooter;

public class ShooterIOPheonix6 {
    public static class ShooterIOInputs {
        public boolean leftMotorConnected;
        public boolean rightMotorConnected;

        public double leftVoltageVolts;
        public double rightVoltageVolts;
        public double leftCurrentAmps;
        public double rightCurrentAmps;
        public double leftVelocityRPS;
        public double rightVelocityRPS;
    }

    public void setVoltage(double voltage) {}

    public void setRPS(double rps) {}

    public void updateInputs(ShooterIOInputs inputs) {}
}
