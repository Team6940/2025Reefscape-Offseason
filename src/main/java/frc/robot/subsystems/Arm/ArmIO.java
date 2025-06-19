package frc.robot.subsystems.Arm;

import java.util.Set;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default public void setVoltage(double voltage) {
    }

    default public void setPosition(double position) {

    }

    default public void zeroArmPostion() {

    }

    @AutoLog
    public class ArmIOInputs {
        public boolean ArmMotorConnected = false;

        public double ArmVoltageVolts;
        public double ArmCurrentAmps;

        public double ArmRotationDegrees = 0.0;
        public double ArmPositionRadians = 0.0;
    }

    default public void updateInputs(ArmIOInputs inputs) {
    }
}
