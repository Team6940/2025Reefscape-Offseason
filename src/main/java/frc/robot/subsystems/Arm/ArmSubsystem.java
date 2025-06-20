package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Library.MUtils;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class ArmSubsystem extends SubsystemBase {

    public static ArmSubsystem m_instance = null;

    public static ArmSubsystem getInstance() {
        return m_instance == null ? m_instance = new ArmSubsystem() : m_instance;
    }

    private final ArmIO io;
    // private final IntakerIOInputsAutoLogged inputs = new
    // IntakerIOInputsAutoLogged();

    private double targetPosition = 0.;

    public ArmSubsystem() {
        if (Robot.isReal()) {
            io = new ArmIOPhoenix6();
        } else {
            // TODO: Implement simulation code here
            io = new ArmIOPhoenix6();
        }
    }

    public void zeroArmPostion() {
        io.zeroArmPostion();
    }

    /**
     * 
     * @param position radians
     */
    public void setPosition(double position) {
        position =  MUtils.numberLimit(ArmConstants.MinRadians, ArmConstants.MaxRadians, position);
        io.setPosition(position);
    }

    boolean IsAtTargetPositon() {
        //needed here
        return true;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void stop() {
        setPosition(0.);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void rotateArm(double rotation){
        rotateArm(rotation);
    }

    @Override
    public void periodic() {
        // processLog();
        processDashboard();
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here
    }

}
