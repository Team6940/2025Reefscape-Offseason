package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default public void setVoltage(double voltage) {

    }

    default public void setPosition(double position) {

    }

    default public void zeroArmPostion() {

    }

    default public void rotateArm(double rotation) {

    }

    @AutoLog
    public class ArmIOInputs {
        public boolean motorConnected = false;
        public double motorVoltageVolts;
        public double motorCurrentAmps;

        public double ArmRotationDegrees = 0.0;
        public double ArmPositionRadians = 0.0;
    }

    default public void updateInputs(ArmIOInputs inputs) {
    }
}
