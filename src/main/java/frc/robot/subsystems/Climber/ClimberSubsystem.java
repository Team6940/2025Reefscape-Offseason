package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Library.MUtils;

public class ClimberSubsystem extends SubsystemBase{
    public static ClimberSubsystem m_Instance = null;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double targetRotation = 0.;
    private double targetLockRPS = 0.;

    public static ClimberSubsystem getInstance() {
        return m_Instance == null ? m_Instance = new ClimberSubsystem() : m_Instance;
    }

    ClimberSubsystem(){
        if(Robot.isReal()){
            io = new ClimberIOPhoenix6();
        }
        else {
            //TODO implement simulation
            io = new ClimberIOPhoenix6();
        }
    };

    public void lockMotorSetRPS(double rps){
        targetLockRPS = rps;
        io.setLockRPS(targetLockRPS); //TODO: Implement lock motor control
    }

    public void setPosition(double rotation){
        rotation = MUtils.numberLimit(ClimberConstants.ClimberMinPos, ClimberConstants.ClimberMaxPos, rotation);
        targetRotation = rotation;
        io.setRotation(rotation);
    }

    public boolean isAtTargetRotation(){
        return Math.abs(targetRotation - inputs.liftMotorPositionRotations) < ClimberConstants.ClimberRotationTolerence;
    }

    public void resetPosition(double rotation){
        io.resetRotation(rotation);
    }

    public double getRotation(){
        return inputs.liftMotorPositionRotations;
    }

    public double getVelocity(){
        return inputs.liftMotorVelocityRPS;
    }

    public void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/targetRotation", targetRotation);
    }

    public void processDashboard(){
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();
    }
}