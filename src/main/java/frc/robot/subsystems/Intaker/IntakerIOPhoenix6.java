package frc.robot.subsystems.Intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakerConstants;

public class IntakerIOPhoenix6 implements IntakerIO {
    private static final TalonFX motor = new TalonFX(IntakerConstants.IntakerMotorID, "canivore");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public IntakerIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = IntakerConstants.IntakerRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = IntakerConstants.kP;
        config.Slot0.kI = IntakerConstants.kI;
        config.Slot0.kD = IntakerConstants.kD;
        config.Slot0.kV = IntakerConstants.kV;
        config.Slot0.kS = IntakerConstants.kS;

        config.MotorOutput.Inverted = IntakerConstants.IntakerInverted;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setRPS(double rps) {
        if(rps == 0){
            motor.stopMotor();
            //rghtMotor.stopMotor();
        }
        motor.setControl(dutycycle.withVelocity(rps));
       // rghtMotor.setControl(dutycycle.withVelocity(rps));
    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getVelocity()
        ).isOK();
        
        inputs.leftVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        //inputs.rghtVoltageVolts = rghtMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        //inputs.rghtCurrentAmps = rghtMotor.getSupplyCurrent().getValueAsDouble();

        inputs.leftVelocityRPS = motor.getVelocity().getValueAsDouble();
        //inputs.rghtVelocityRPS = rghtMotor.getVelocity().getValueAsDouble();
    }


}
