package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOPhoenix6 implements ElevatorIO{
    private static TalonFX leftMotor, rghtMotor;
    private static MotionMagicVoltage m_request = new MotionMagicVoltage(0.);

    public ElevatorIOPhoenix6() {
        motorConfig();
    }

    private void motorConfig() {
     
        leftMotor = new TalonFX(ElevatorConstants.leftMotorID, "rio");
        rghtMotor = new TalonFX(ElevatorConstants.rghtMotorID, "rio");

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Voltage.PeakForwardVoltage = 12.;
        configs.Voltage.PeakReverseVoltage = -6.5;          //This can slow down its going down, reducing impact on the base tube.
        configs.Feedback.SensorToMechanismRatio = ElevatorConstants.MotorToRollerRatio;
        configs.MotorOutput.DutyCycleNeutralDeadband = 0.02; 

        configs.Slot0.kG = ElevatorConstants.kG;
        configs.Slot0.kS = ElevatorConstants.kS;
        configs.Slot0.kV = ElevatorConstants.kV;
        configs.Slot0.kP = ElevatorConstants.kP;
        configs.Slot0.kI = ElevatorConstants.kI;
        configs.Slot0.kD = ElevatorConstants.kD;

        configs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        configs.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MaxVelocity;
        configs.MotionMagic.MotionMagicAcceleration = ElevatorConstants.Acceleration;

        configs.MotorOutput.Inverted = ElevatorConstants.LeftInverted;
        leftMotor.getConfigurator().apply(configs);

        configs.MotorOutput.Inverted = ElevatorConstants.RghtInverted;
        rghtMotor.getConfigurator().apply(configs);

        zeroHeight();
    }

    @Override
    public void setHeight(double targetHeight){
        leftMotor.setControl(m_request.withPosition(targetHeight / ElevatorConstants.RollerRoundToMeters));
        rghtMotor.setControl(m_request.withPosition(targetHeight / ElevatorConstants.RollerRoundToMeters));
    }

    @Override
    public void resetHeight(double _height){
        leftMotor.setPosition(_height / ElevatorConstants.RollerRoundToMeters);
        rghtMotor.setPosition(_height / ElevatorConstants.RollerRoundToMeters);
    }

    @Override
    public void setVoltage(double _Voltage){
        leftMotor.setVoltage(_Voltage);
        rghtMotor.setVoltage(_Voltage);
    }


    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.leftConnected = leftMotor.isAlive();
        inputs.rghtConnected = rghtMotor.isAlive();
        inputs.leftMotorPositionRotations = leftMotor.getPosition().getValueAsDouble();
        inputs.rghtMotorPositionRotations = rghtMotor.getPosition().getValueAsDouble();
        inputs.leftMotorVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
        inputs.rghtMotorVelocityRPS = rghtMotor.getVelocity().getValueAsDouble();
        inputs.leftVoltageVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.rghtVoltageVolts = rghtMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rghtCurrentAmps = rghtMotor.getSupplyCurrent().getValueAsDouble();

        inputs.ElevatorHeight=inputs.leftMotorPositionRotations*ElevatorConstants.RollerRoundToMeters;
        inputs.ElevatorVelocity=inputs.leftMotorVelocityRPS;
    }
}
