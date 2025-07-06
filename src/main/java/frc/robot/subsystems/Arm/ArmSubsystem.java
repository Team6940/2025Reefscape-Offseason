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
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

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
     * @param position degs
     */
    public void setPosition(double position) {
        targetPosition = MUtils.numberLimit(ArmConstants.MinDegs, ArmConstants.MaxDegs, position);
        io.setPosition(targetPosition);
    }

    public boolean isAtTargetPositon() {
        return MathUtil.isNear(targetPosition, inputs.ArmPositionDegs, ArmConstants.ArmPositionToleranceDegs);
    }

    public boolean isAtSecuredPosition() {
        return Math.abs(getArmPosition()) > ArmConstants.SecuredPosition;// TODO
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void reset() {
        setPosition(0.);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    /**
     * The current position of the arm in degrees.
     * 
     * @return the current position of the arm in degrees.
     */
    public double getArmPosition() {
        return inputs.ArmPositionDegs;
    }

    /**
     * Rotates the arm by the specified amount, CCW positive.
     * 
     * @param angleDegs in degrees
     */
    public void rotateArm(double angleDegs) {
        double position = inputs.ArmPositionDegs + angleDegs;
        setPosition(position);
    }

    /**
     * Try to stay at the exact position the time this method is called.
     * In effect, this equals to setPosition(getArmPosition()).
     * 
     * @see #stop()
     */
    public void stay() {
        setPosition(getArmPosition());
    }

    /**
     * Try to stop the arm from moving.
     * Does <b>not</b> prevent swinging.
     * 
     * @see #stay()
     */
    public void stop() {
        io.setVoltage(0);
    }

    public void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/TargetPosition", targetPosition);
        Logger.recordOutput("Arm/Position", inputs.ArmPositionDegs);
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
