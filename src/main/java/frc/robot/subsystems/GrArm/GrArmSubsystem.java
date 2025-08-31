package frc.robot.subsystems.GrArm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Library.MUtils;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.GrArmConstants;

public class GrArmSubsystem extends SubsystemBase {

    public static GrArmSubsystem m_instance = null;

    public static GrArmSubsystem getInstance() {
        return m_instance == null ? m_instance = new GrArmSubsystem() : m_instance;
    }

    private final GrArmIO io;
    private final GrArmIOInputsAutoLogged inputs = new GrArmIOInputsAutoLogged();

    private double targetPosition = 0.;

    public GrArmSubsystem() {
        if (Robot.isReal()) {
            io = new GrArmIOPhoenix6();
        } else {
            // TODO: Implement simulation code here
            io = new GrArmIO(){};
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
        targetPosition = MUtils.numberLimit(GrArmConstants.MinDegs, GrArmConstants.MaxDegs, position);
        io.setPosition(targetPosition);
    }

    boolean IsAtTargetPositon() {
        return MathUtil.isNear(targetPosition, inputs.GrArmPositionRadians, GrArmConstants.GrArmPositionToleranceDegs);
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

    public void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("GrArm", inputs);
        Logger.recordOutput("GrArm/TargetPosition", targetPosition);
        // TODO Logger
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here
    }

}
