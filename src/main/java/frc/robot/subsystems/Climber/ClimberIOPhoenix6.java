package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ClimberConstants;

public class ClimberIOPhoenix6 implements ClimberIO{
    private static TalonFX liftMotor = new TalonFX(ClimberConstants.ClimberliftMotorID, "canivore"); //TODO change to canivore
    // private static TalonFX lockMotor = new TalonFX(ClimberConstants.ClimberlockMotorID, "rio"); //TODO change to canivore

    private static final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.);
    //private static final VelocityVoltage dutycycle = new VelocityVoltage(0);

    ClimberIOPhoenix6(){
        motorConfig();
        liftMotor.setPosition(0.);
        // lockMotor.setPosition(0.); //TODO
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

        configs.MotionMagic.MotionMagicAcceleration = ClimberConstants.ClimberAcceleration;
        configs.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.ClimberMaxVelocity;

        liftMotor.getConfigurator().apply(configs);
        // lockMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setRotation(double rotation){
        liftMotor.setControl(positionVoltage.withPosition(rotation));
    }

    @Override
    public void resetRotation(double rotation){
        liftMotor.setPosition(rotation);
    }

    @Override
    public void zeroRotation(){
        resetRotation(0);
    }

    // @Override
    // public void setLockRPS(double rps){
    //     lockMotor.setControl(dutycycle.withVelocity(rps)); //TODO
    // }

    @Override
    public void updateInputs(ClimberIOInputs inputs){
        inputs.liftMotorConnected = BaseStatusSignal.refreshAll(
            liftMotor.getVelocity(),
            liftMotor.getMotorVoltage(),
            liftMotor.getPosition()
        ).isOK();

        inputs.liftMotorVelocityRPS = liftMotor.getVelocity().getValueAsDouble();
        inputs.liftMotorPositionRotations = liftMotor.getPosition().getValueAsDouble();
        inputs.liftMotorVoltageVolts = liftMotor.getMotorVoltage().getValueAsDouble();

        // inputs.lockMotorConnected = BaseStatusSignal.refreshAll(
        //     lockMotor.getVelocity(),
        //     lockMotor.getMotorVoltage(),
        //     lockMotor.getPosition()
        // ).isOK();

        // inputs.lockMotorVelocityRPS = lockMotor.getVelocity().getValueAsDouble();
        // inputs.lockMotorPositionRotations = lockMotor.getPosition().getValueAsDouble();
        // inputs.lockMotorVoltageVolts = lockMotor.getMotorVoltage().getValueAsDouble();
    }
}
