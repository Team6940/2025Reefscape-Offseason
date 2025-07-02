package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    public void setVoltage(double voltage);

    /**
     * @param position in degrees
     */
    public void setPosition(double position);

    /**
     * Resets the encoder position to desired position.
     * @param position in degrees
     */
    public void resetPosition(double position);

    /**
     * Resets the encoder position to 0.0 degrees.
     * in effect io.resetPosition(0.0);
     */
    public void zeroArmPostion();

    @AutoLog
    public class ArmIOInputs {
        public boolean motorConnected = false;
        public double motorVoltageVolts;
        public double motorCurrentAmps;

        public boolean encoderConnected = false;
        public double encoderPositionDegrees;
        public enum EncoderMagnetHealth {
            GOOD,
            RISKY,
            BAD,
            INVALID
        }

        public EncoderMagnetHealth encoderMagnetHealth = EncoderMagnetHealth.BAD;
        public double ArmPositionDegs = 0.0;
    }

    default public void updateInputs(ArmIOInputs inputs) {
    }
}
