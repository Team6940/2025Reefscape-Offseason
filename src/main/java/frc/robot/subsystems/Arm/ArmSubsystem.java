package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class ArmSubsystem extends SubsystemBase{
    
    public static ArmSubsystem m_instance = null;

    public static ArmSubsystem getInstance() {
        return m_instance == null? m_instance = new ArmSubsystem() : m_instance;
    }

    private final ArmIO io;
    // private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;

    public ArmSubsystem(){
        if(Robot.isReal()){
            io = new ArmIOPhoenix6();
        }
        else{
            //TODO: Implement simulation code here
            io = new ArmIOPhoenix6();
        }
    }
    /**
     * 
     * @param position radians
     */
    public void setPosition(double position){
        targetRPS = position;
        io.setPosition(position);
    }
   
    public double getTargetRPS(){
        return targetRPS;
    }

    // public boolean IsAtTargetRPS(){
    //      return MathUtil.isNear(targetRPS, (inputs.leftVelocityRPS + inputs.rghtVelocityRPS) / 2., GroundIntakerConstants.intakerVelocityToleranceRPS);
    // }

    public void stop(){
        setPosition(0.);
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
