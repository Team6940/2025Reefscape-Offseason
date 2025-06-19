package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;

public class IndexerPhoenix6 implements IndexerIO {
    private static final TalonFX Motor = new TalonFX(IndexerConstants.IndexerMotorID, "canivore");

    private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    public IndexerPhoenix6() {
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
        config.MotorOutput.Inverted = IndexerConstants.IndexerInverted;
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

    public void updateInputs(IndexerIOInputs IndexerInputs) {
        IndexerInputs.IndexerMotorConnected = BaseStatusSignal.refreshAll(
            Motor.getMotorVoltage(),
            Motor.getSupplyCurrent(),
            Motor.getVelocity()
        ).isOK();
        
        IndexerInputs.IndexerVoltageVolts = Motor.getMotorVoltage().getValueAsDouble();
        IndexerInputs.IndexerCurrentAmps = Motor.getSupplyCurrent().getValueAsDouble();
        IndexerInputs.IndexerVelocityRPS = Motor.getVelocity().getValueAsDouble();
    }


}
