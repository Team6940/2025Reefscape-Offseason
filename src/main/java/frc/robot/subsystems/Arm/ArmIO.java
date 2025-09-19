package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default public void setVoltage(double voltage){}

    /**
     * @param position in degrees
     */
    default public void setPosition(double position){}

    /**
     * Resets the encoder position to desired position.
     * @param position in degrees
     */
    default public void resetPosition(double position){}

    /**
     * Resets the encoder position to 0.0 degrees.
     * in effect io.resetPosition(0.0);
     */
    default public void zeroArmPostion(){}

    @AutoLog
    public class ArmIOInputs {
        public boolean motorConnected = false;
        public double motorVoltageVolts;
        public double motorCurrentAmps;

        public double encoderPositionDegs = 0.0;

        public boolean encoderConnected = false;
        public enum EncoderMagnetHealth {
            GOOD,
            RISKY,
            BAD,
            INVALID
        }

        public EncoderMagnetHealth encoderMagnetHealth = EncoderMagnetHealth.BAD;
        public double armPositionDegs = 0.0;
    }

    default public void updateInputs(ArmIOInputs inputs) {
    }
}
