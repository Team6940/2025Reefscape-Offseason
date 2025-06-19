package frc.robot.subsystems.GrArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Library.MUtils;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

public class GrArmSubsystem extends SubsystemBase {

    public static GrArmSubsystem m_instance = null;

    public static GrArmSubsystem getInstance() {
        return m_instance == null ? m_instance = new GrArmSubsystem() : m_instance;
    }

    private final GrArmIO io;
    // private final IntakerIOInputsAutoLogged inputs = new
    // IntakerIOInputsAutoLogged();

    private double targetPosition = 0.;

    public GrArmSubsystem() {
        if (Robot.isReal()) {
            io = new GrArmIOPhoenix6();
        } else {
            // TODO: Implement simulation code here
            io = new GrArmIOPhoenix6();
        }
    }

    public void zeroGrArmPostion() {
        io.zeroGrArmPostion();
    }

    /**
     * 
     * @param position radians
     */
    public void setPosition(double position) {
        position =  MUtils.numberLimit(GrArmConstants.MinRadians, GrArmConstants.MaxRadians, position);
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

    @Override
    public void periodic() {
        // processLog();
        processDashboard();
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here
    }

}
