package frc.robot.subsystems.Climber;

public class ClimberPheonix6 {
    public static class ClimberIOInputs {
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

    public void updateInputs(ClimberIOInputs inputs) {}
}
