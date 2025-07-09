package frc.robot.subsystems.GrArm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants.GrArmConstants;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

public class GrArmIOPhoenix6 implements GrArmIO {
    private static TalonFX motor;

    private static MotionMagicVoltage m_request = new MotionMagicVoltage(0.);
    public GrArmIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        motor = new TalonFX(GrArmConstants.GrArmMotorID, "rio");
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GrArmConstants.GrArmRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = GrArmConstants.kP;
        config.Slot0.kI = GrArmConstants.kI;
        config.Slot0.kD = GrArmConstants.kD;
        config.Slot0.kV = GrArmConstants.kV;
        config.Slot0.kS = GrArmConstants.kS;
        config.Slot0.kG = GrArmConstants.kG;

        config.MotorOutput.Inverted = GrArmConstants.Inverted;

        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.MotionMagic.MotionMagicCruiseVelocity = GrArmConstants.MaxVelocity;
        config.MotionMagic.MotionMagicAcceleration = GrArmConstants.Acceleration;

        //config.MotorOutput.DutyCycleNeutralDeadband = GrArmConstants.Deadband;
        motor.setPosition(0.25);//1/4rotation, which means 90degs

        motor.getConfigurator().apply(config);

            //zeroGrArmPostion();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double position) {
        // if (position == 0) {
        //     motor.stopMotor();
        // }
        motor.setControl(m_request.withPosition(Units.degreesToRotations(position)));
    }

    public void updateInputs(GrArmIOInputs GrArmInputs) {
        GrArmInputs.motorConnected = BaseStatusSignal.refreshAll(
                motor.getMotorVoltage(),
                motor.getSupplyCurrent(),
                motor.getVelocity()).isOK();

        GrArmInputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        GrArmInputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        GrArmInputs.GrArmRotationDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
        //GrArmInputs.GrArmPositionRadians = GrArmInputs.GrArmRotationDegrees * Math.PI / 180.0;

    }

}
