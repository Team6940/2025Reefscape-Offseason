package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.GroundIntakerConstants;

public class ArmIOPhoenix6 implements ArmIO {
    private static final TalonFX Motor = new TalonFX(GroundIntakerConstants.IntakerLeftMotorID, "canivore");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public ArmIOPhoenix6() {
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
        Motor.getConfigurator().apply(config);

    }

    @Override
    public void setVoltage(double voltage) {
        Motor.setVoltage(voltage);
    }

    @Override
    public void setRPS(double rps) {
        if(rps == 0){
            Motor.stopMotor();
        }
        Motor.setControl(dutycycle.withVelocity(rps));
    }

    public void updateInputs(ArmIOInputs ArmInputs) {
        ArmInputs.ArmMotorConnected = BaseStatusSignal.refreshAll(
            Motor.getMotorVoltage(),
            Motor.getSupplyCurrent(),
            Motor.getVelocity()
        ).isOK();
        
        ArmInputs.ArmVoltageVolts = Motor.getMotorVoltage().getValueAsDouble();
        ArmInputs.ArmCurrentAmps = Motor.getSupplyCurrent().getValueAsDouble();
        ArmInputs.ArmVelocityRPS = Motor.getVelocity().getValueAsDouble();
    }


}
