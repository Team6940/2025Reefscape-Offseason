package frc.robot.subsystems.GrArm;

import java.util.Set;

import org.littletonrobotics.junction.AutoLog;

public interface GrArmIO {
    default public void setVoltage(double voltage) {

    }

    default public void setPosition(double position) {

    }

    default public void zeroGrArmPostion() {

    }

    @AutoLog
    public class GrArmIOInputs {
        public boolean GrArmMotorConnected = false;

        public double GrArmVoltageVolts;
        public double GrArmCurrentAmps;

        public double GrArmPositionRadians = 0.;
        public double GrArmRotationDegrees = 0.;
    }

    default public void updateInputs(GrArmIOInputs inputs) {
    }
}
