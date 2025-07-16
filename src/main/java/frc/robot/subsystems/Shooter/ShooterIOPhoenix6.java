package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOPhoenix6 implements ShooterIO{
    private static final TalonFX motor = new TalonFX(ShooterConstants.ShooterMotorID, "canivore");
    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public ShooterIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = ShooterConstants.ShooterInverted;
        config.Feedback.SensorToMechanismRatio = ShooterConstants.ShooterRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.CurrentLimits.SupplyCurrentLimit=ShooterConstants.SupplyCurrentLimit;
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kS = ShooterConstants.kS;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void setRPS(double rps) {
        if(rps == 0){
            motor.stopMotor();
        }
        motor.setControl(dutycycle.withVelocity(rps));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getVelocity()
        ).isOK();

        inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.shooterVelocityRPS = motor.getVelocity().getValueAsDouble();
    }
    
}
