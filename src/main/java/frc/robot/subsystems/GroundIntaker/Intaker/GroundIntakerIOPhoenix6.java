package frc.robot.subsystems.GroundIntaker.Intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.GroundIntakerConstants;

public class GroundIntakerIOPhoenix6 implements GroundIntakerIO {
    private static final TalonFX leftMotor = new TalonFX(GroundIntakerConstants.IntakerLeftMotorID, "canivore");
    private static final TalonFX rghtMotor = new TalonFX(GroundIntakerConstants.IntakerRghtMotorID, "canivore");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public GroundIntakerIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = GroundIntakerConstants.IntakerRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = GroundIntakerConstants.kP;
        config.Slot0.kI = GroundIntakerConstants.kI;
        config.Slot0.kD = GroundIntakerConstants.kD;
        config.Slot0.kV = GroundIntakerConstants.kV;
        config.Slot0.kS = GroundIntakerConstants.kS;

        config.MotorOutput.Inverted = GroundIntakerConstants.LeftInverted;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = GroundIntakerConstants.RghtInverted;
        rghtMotor.getConfigurator().apply(config);

    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rghtMotor.setVoltage(voltage);
    }

    @Override
    public void setRPS(double rps) {
        if(rps == 0){
            leftMotor.stopMotor();
            rghtMotor.stopMotor();
        }
        leftMotor.setControl(dutycycle.withVelocity(rps));
        rghtMotor.setControl(dutycycle.withVelocity(rps));
    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
            leftMotor.getMotorVoltage(),
            leftMotor.getSupplyCurrent(),
            leftMotor.getVelocity()
        ).isOK();
        inputs.rghtMotorConnected = BaseStatusSignal.refreshAll(
            rghtMotor.getMotorVoltage(),
            rghtMotor.getSupplyCurrent(),
            rghtMotor.getVelocity()
        ).isOK();
        
        inputs.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rghtVoltageVolts = rghtMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rghtCurrentAmps = rghtMotor.getSupplyCurrent().getValueAsDouble();

        inputs.leftVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
        inputs.rghtVelocityRPS = rghtMotor.getVelocity().getValueAsDouble();
    }


}
