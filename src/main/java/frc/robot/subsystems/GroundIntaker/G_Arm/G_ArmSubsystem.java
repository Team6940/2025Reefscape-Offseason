package frc.robot.subsystems.GroundIntaker.G_Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.GroundIntakerConstants;

public class G_ArmSubsystem extends SubsystemBase{
    
    public static G_ArmSubsystem m_instance;
    public static G_ArmSubsystem getInstance() {
        return m_instance == null? m_instance = new G_ArmSubsystem() : m_instance;
    }

    private final G_ArmIO io;
    // private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;

    public G_ArmSubsystem(){
        if(Robot.isReal()){
            io = new G_ArmIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new G_ArmIOPhoenix6();
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
