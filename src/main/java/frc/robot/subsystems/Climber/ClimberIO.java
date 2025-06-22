package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    public void setRotation(double rotation);

    public void resetRotation(double rotation);

    default public void zeroRotation() {
        resetRotation(0.);
    };

    @AutoLog
    public class ClimberIOInputs{

        public boolean motorConnected;
        public double motorVoltageVolts;
        public double climberPositionRotations;
        public double climberVelocityRPS;

        // public double ClimberHeight;
        // public double ClimberVelocity;
    }

    public void updateInputs(ClimberIOInputs inputs);
}
