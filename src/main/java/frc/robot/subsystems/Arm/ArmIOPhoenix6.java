package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import frc.robot.Constants.ArmConstants;
import frc.robot.Library.team1678.math.Conversions;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs.EncoderMagnetHealth;

public class ArmIOPhoenix6 implements ArmIO {
    private static TalonFX motor;
    private static CANcoder encoder;

    private static MotionMagicVoltage m_request = new MotionMagicVoltage(0.);

    public ArmIOPhoenix6() {
        encoderConfig();
        motorConfig();      //Motor is better configured after encoder because it might be attached to the encoder;
    }

    private void motorConfig() {
        motor = new TalonFX(ArmConstants.ArmMotorID, "canivore");
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //U no longer needs ratio because u paid for the encoder

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = ArmConstants.kP;
        config.Slot0.kI = ArmConstants.kI;
        config.Slot0.kD = ArmConstants.kD;
        config.Slot0.kV = ArmConstants.kV;
        config.Slot0.kS = ArmConstants.kS;
        config.Slot0.kG = ArmConstants.kG;

        //U no longer needs InvertedValue because u paid for the encoder, see encoderConfig()

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MaxVelocity;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.Acceleration;

        config.Feedback.SensorToMechanismRatio = ArmConstants.encoderToMechanismRatio;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = ArmConstants.ArmEncoderID;

        motor.getConfigurator().apply(config);

    }

    private void encoderConfig() {
        encoder = new CANcoder(ArmConstants.ArmEncoderID, "canivore");
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = Units.degreesToRotations(ArmConstants.EncoderOffsetDegrees);
        config.MagnetSensor.SensorDirection = ArmConstants.EncoderDirection;
        encoder.getConfigurator().apply(config);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    /**
     * @param position in degrees
     */
    public void setPosition(double position) {
        motor.setControl(m_request.withPosition(Units.degreesToRotations(position)));
    }

    @Override
    public void resetPosition(double position) {
        encoder.setPosition(Units.degreesToRotations(position));
    }

    @Override
    public void zeroArmPostion() {
        encoder.setPosition(0.);
    }

    public void updateInputs(ArmIOInputs ArmInputs) {
        ArmInputs.motorConnected = BaseStatusSignal.refreshAll(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getVelocity()
        ).isOK();

        ArmInputs.encoderConnected = BaseStatusSignal.refreshAll(
            encoder.getAbsolutePosition(),
            encoder.getMagnetHealth()
        ).isOK();

        switch (encoder.getMagnetHealth().getValue()) {
            case Magnet_Green:
                ArmInputs.encoderMagnetHealth = EncoderMagnetHealth.GOOD;
                break;
            case Magnet_Orange:
                ArmInputs.encoderMagnetHealth = EncoderMagnetHealth.RISKY;
                break;
            case Magnet_Red:
                ArmInputs.encoderMagnetHealth = EncoderMagnetHealth.BAD;
                break;
            case Magnet_Invalid:
                ArmInputs.encoderMagnetHealth = EncoderMagnetHealth.INVALID;
                break;
            default:
                break;
        }

        ArmInputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        ArmInputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        ArmInputs.armPositionDegs = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

        ArmInputs.encoderPositionDegs = Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble());
        
    }
}
