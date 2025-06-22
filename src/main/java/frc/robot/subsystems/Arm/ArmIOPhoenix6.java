package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants.ArmConstants;

public class ArmIOPhoenix6 implements ArmIO {
    private static TalonFX motor;

    private static MotionMagicVoltage m_request = new MotionMagicVoltage(0.);

    public ArmIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        motor = new TalonFX(ArmConstants.ArmMotorID, "rio");
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ArmConstants.ArmRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = ArmConstants.kP;
        config.Slot0.kI = ArmConstants.kI;
        config.Slot0.kD = ArmConstants.kD;
        config.Slot0.kV = ArmConstants.kV;
        config.Slot0.kS = ArmConstants.kS;
        config.Slot0.kG = ArmConstants.kG;

        config.MotorOutput.Inverted = ArmConstants.Inverted;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MaxVelocity;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.Acceleration;

        motor.getConfigurator().apply(config);

        zeroArmPostion();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position) {
        if (position == 0) {
            motor.stopMotor();
        }
        motor.setControl(m_request.withPosition(position));
    }

    @Override
    public void rotateArm(double rotation) {
        double position = motor.getPosition().getValueAsDouble();
        position += rotation;
        position = Math.max(ArmConstants.MinRadians, Math.min(ArmConstants.MaxRadians, position));
        setPosition(position);
    }

    public void updateInputs(ArmIOInputs ArmInputs) {
        ArmInputs.ArmMotorConnected = BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getVelocity()).isOK();

        ArmInputs.ArmVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        ArmInputs.ArmCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        ArmInputs.ArmRotationDegrees = motor.getPosition().getValueAsDouble();
        ArmInputs.ArmPositionRadians = ArmInputs.ArmRotationDegrees * Math.PI / 180.0;

    }

}
