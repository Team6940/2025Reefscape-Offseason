package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    public void setRotation(double rotation);

    public void resetRotation(double rotation);

    default public void zeroRotation() {
        resetRotation(0.);
    };

    default public void setLockRPS(double rps){}

    @AutoLog
    public class ClimberIOInputs{

        public boolean liftMotorConnected;
        public double liftMotorVoltageVolts;
        public double liftMotorPositionRotations;
        public double liftMotorVelocityRPS;

        public double lockMotorCurrentAmps;
        public boolean lockMotorConnected;
        public double lockMotorVoltageVolts;
        public double lockMotorPositionRotations;
        public double lockMotorVelocityRPS;

        // public double ClimberHeight;
        // public double ClimberVelocity;
    }

    public void updateInputs(ClimberIOInputs inputs);
}
