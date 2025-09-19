package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest.Idle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.Constants.IntakerConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakerSubsystem extends SubsystemBase{
    public static IntakerSubsystem m_instance;
    public static IntakerSubsystem getInstance() {
        return m_instance == null? m_instance = new IntakerSubsystem() : m_instance;
    }

    // private IntakerState state;

    // public enum IntakerState {
    //     IDLE,
    //     READY,
    //     GRABBING
    // }

    private final IntakerIO io;
    private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;

    public IntakerSubsystem(){
        if(Robot.isReal()){
            io = new IntakerIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new IntakerIO(){};
        }
    }

    // public IntakerState getIntakerState() {
    //     return state;
    // }

    // public boolean isReady(){
    //     return getIntakerState() == IntakerState.READY;
    // }

    //     public void intakerStateUpdate(){
    //     double current = inputs.motorCurrentAmps;
    //     if(current<IntakerConstants.IntakerFreeSpinCurrentThreshold){
    //         state=IntakerState.IDLE;
    //     }
    //     else if(current<IntakerConstants.IntakerHoldingCurrentThreshold){
    //         state=IntakerState.READY;
    //     }
    //     else{
    //         state=IntakerState.GRABBING;
    //     }
    // }

    public void setRPS(double rps){
        targetRPS = rps;
        io.setRPS(rps);
    }

    public boolean isAtTargetRPS(){
        return MathUtil.isNear(targetRPS, inputs.intakerVelocityRPS, IntakerConstants.IntakerVelocityToleranceRPS);
    }

    public void stop(){
        setRPS(0.);
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();
    }

    public void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Intaker", inputs);
        Logger.recordOutput("Intaker/TargetRPS", targetRPS);
        Logger.recordOutput("Intaker/IsAtTargetRPS", isAtTargetRPS());
    }

    private void processDashboard(){
        //TODO: Implement dashboard code here
    }

}
