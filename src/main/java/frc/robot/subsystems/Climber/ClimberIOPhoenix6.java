package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOPhoenix6 implements ClimberIO{
    private static TalonFX motor = new TalonFX(ClimberConstants.ClimberMotorID, "rio");

    private static final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.);

    ClimberIOPhoenix6(){
        motorConfig();
        motor.setPosition(0.);
    }

    private void motorConfig(){
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = ClimberConstants.ClimberInverted;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Voltage.PeakForwardVoltage = 12.;
        configs.Voltage.PeakReverseVoltage = -6.5;

        configs.Feedback.SensorToMechanismRatio = ClimberConstants.ClimberRatio;

        configs.Slot0.kP = ClimberConstants.ClimberkP;
        configs.Slot0.kD = ClimberConstants.ClimberkD;
        configs.Slot0.kI = ClimberConstants.ClimberkI;
        configs.Slot0.kS = ClimberConstants.ClimberkS;
        configs.Slot0.kG = ClimberConstants.ClimberkG;
        configs.Slot0.kV = ClimberConstants.ClimberkV;

        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        configs.MotionMagic.MotionMagicAcceleration = ClimberConstants.Acceleration;
        configs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.MaxVelocity;

        motor.getConfigurator().apply(configs);
    }

    @Override
    public void setRotation(double rotation){
        motor.setControl(positionVoltage.withPosition(rotation));
    }

    @Override
    public void resetRotation(double rotation){
        motor.setPosition(rotation);
    }

    @Override
    public void zeroRotation(){
        resetRotation(0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.motorConnected = BaseStatusSignal.refreshAll(
            motor.getVelocity(),
            motor.getMotorVoltage(),
            motor.getPosition()
        ).isOK();

        inputs.climberVelocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.climberPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVoltageVolts = motor.getMotorVoltage().getValueAsDouble();
    }
}
