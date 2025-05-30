package frc.robot.subsystems.GroundIntaker.Transmission;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.GroundIntakerConstants;

public class TransmissionSubsystem extends SubsystemBase{
    
    public static TransmissionSubsystem m_instance;
    public static TransmissionSubsystem getInstance() {
        return m_instance == null? m_instance = new TransmissionSubsystem() : m_instance;
    }

    private final TransmissionIO io;
    // private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;

    public TransmissionSubsystem(){
        if(Robot.isReal()){
            io = new TransmissionIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new TransmissionIOPhoenix6();
        }
    }

    public void setRPS(double rps){
        targetRPS = rps;
        io.setRPS(rps);
    }

    // public boolean IsAtTargetRPS(){
    //      return MathUtil.isNear(targetRPS, (inputs.leftVelocityRPS + inputs.rghtVelocityRPS) / 2., GroundIntakerConstants.intakerVelocityToleranceRPS);
    // }

    public void stop(){
        setRPS(0.);
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        // processLog();
        processDashboard();
    }

    // private void processLog(){
    //     io.updateInputs(inputs);
    //     Logger.processInputs("Intaker", inputs);
    //     Logger.recordOutput("Intaker/TargetRPS", targetRPS);
    //     Logger.recordOutput("Intaker/IsAtTargetRPS", IsAtTargetRPS());
    // }

    private void processDashboard(){
        //TODO: Implement dashboard code here
    }

}
