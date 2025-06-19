package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem{
    public static ShooterSubsystem m_Instance;
    public static ShooterSubsystem getInstance(){
        return m_Instance == null? m_Instance = new ShooterSubsystem() : m_Instance;
    }

    private final ShooterIO io;
    //private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double targetRPS = 0;

    public ShooterSubsystem() {
        if(Robot.isReal()){
            io = new ShooterIOPhoenix6();
            // io = new ShooterIOEmpty();
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
        //return MathUtil.isNear(targetRPS, inputs.shooterVelocityRPS, ShooterConstants.ShooterSpeedTolerence);
        return true;
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
        COMING_IN,
        READY,
        COMING_OUT
    }

    //public ShooterState getCoralState() {
        // if (inputs.motorConnected == false) { //TODO:用电流判断吸球装置
        //     return ShooterState.READY;
        // }
        // return ShooterState.IDLE;
   // }

    // public boolean isReady() {
    //     return getCoralState() == ShooterState.READY;
    // }

    // @Override
    // public void periodic() {
    //     processLog();
    //     processDashboard();
        
    //     SmartDashboard.putString("status", getCoralState().toString());
    // }

    private void processLog() {
    //     io.updateInputs(inputs);
    //     Logger.processInputs("Shooter", inputs);
    //     Logger.recordOutput("Shooter/TargetRPS", targetRPS);
    //     Logger.recordOutput("Shooter/IsAtTargetRPS", IsAtTargetRps());
    }

    private void processDashboard() {
        //TODO: Implement dashboard code here

    }

    public double getShootRPS(int level){
        return ShooterConstants.ShooterShootRPSs[level];
    }

}
