package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    public static ShooterSubsystem m_Instance;
    public static ShooterSubsystem getInstance(){
        return m_Instance == null? m_Instance = new ShooterSubsystem() : m_Instance;
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetRPS = 0;

    private ShooterState state;

    public ShooterSubsystem() {
        if(Robot.isReal()){
            io = new ShooterIOPhoenix6();
            //io = new ShooterIOEmpty();
        }
        else{
            //TODO: Implement simulation code here
            io = new ShooterIOPhoenix6();
        }
    }

    public void setRPS(double rps) {
        targetRPS = rps;
        io.setRPS(rps);
    }

    public boolean IsAtTargetRps() {
        return MathUtil.isNear(targetRPS, inputs.shooterVelocityRPS, ShooterConstants.ShooterSpeedTolerence);
    }

    public void stop() {
        targetRPS = 0;
        io.setRPS(0);
    }

    public void setVoltage(double voltage) {
        targetRPS = 0;
        io.setVoltage(voltage);
    }

    public enum ShooterState{
        IDLE,
        FREE_SPINNING,
        READY,
        GRABBING
    }

    public ShooterState getShooterState() {
       return state;
   }

    public boolean isReady() {
        return getShooterState() == ShooterState.READY;
    }

    public void shooterStateUpdate(){
        double current = inputs.motorCurrentAmps;
        if(current<ShooterConstants.ShooterFreeSpinCurrentThreshold){
            state=ShooterState.IDLE;
        }
        else if(current<ShooterConstants.ShooterReadyCurrentThreshold){
            state=ShooterState.FREE_SPINNING;
        }
        else if(current<ShooterConstants.ShooterGrabbingCurrentThreshold){
            state=ShooterState.READY;
        }
        else{
            state=ShooterState.GRABBING;
        }

    }

    @Override
    public void periodic() {
        shooterStateUpdate();
        processLog();
        processDashboard();
        
        SmartDashboard.putString("status", getShooterState().toString());
    }

    private void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/TargetRPS", targetRPS);
        Logger.recordOutput("Shooter/IsAtTargetRPS", IsAtTargetRps());
    }

    private void processDashboard() {
        //TODO: Implement dashboard code here

    }

    public double getShootRPS(int level){
        return ShooterConstants.ShooterShootRPSs[level];
    }

}
