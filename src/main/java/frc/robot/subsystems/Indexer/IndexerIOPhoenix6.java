package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;

public class IndexerIOPhoenix6 implements IndexerIO {
    private static final TalonFX leftMotor = new TalonFX(IndexerConstants.IndexerLeftMotorID, "canivore");
    private static final TalonFX rghtMotor = new TalonFX(IndexerConstants.IndexerRghtMotorID, "canivore");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public IndexerIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.IndexerRatio;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.Slot0.kP = IndexerConstants.kP;
        config.Slot0.kI = IndexerConstants.kI;
        config.Slot0.kD = IndexerConstants.kD;
        config.Slot0.kV = IndexerConstants.kV;
        config.Slot0.kS = IndexerConstants.kS;

        config.MotorOutput.Inverted = IndexerConstants.LeftInverted;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = IndexerConstants.RghtInverted;
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
    public void updateInputs(IndexerIOInputs inputs) {
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
