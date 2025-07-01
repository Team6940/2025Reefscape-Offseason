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
     * @param position radians
     */
    public void setPosition(double position) {
        targetPosition = MUtils.numberLimit(ArmConstants.MinRadians, ArmConstants.MaxRadians, position);
        io.setPosition(targetPosition);
    }

    public boolean isAtTargetPositon() {
        return MathUtil.isNear(targetPosition, inputs.ArmPositionRadians, ArmConstants.ArmPositionToleranceRadians);
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

    public double getArmPosition() {
        return inputs.ArmPositionRadians;
    }

    public void rotateArm(double rotation) {
        io.rotateArm(rotation);
    }

    public void processLog() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        Logger.recordOutput("Arm/TargetPosition", targetPosition);
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
