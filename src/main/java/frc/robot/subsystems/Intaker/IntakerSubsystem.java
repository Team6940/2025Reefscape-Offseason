package frc.robot.subsystems.Intaker;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.GeneralConstants.IntakerConstants;
import frc.robot.constants.SimConstants.IntakerSimConstants;
import frc.robot.util.Simulation.ComponentPublisher;

public class IntakerSubsystem extends SubsystemBase{
    public static IntakerSubsystem m_instance;
    public static IntakerSubsystem getInstance() {
        return m_instance == null? m_instance = new IntakerSubsystem() : m_instance;
    }

    private final IntakerIO io;
    private final IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private double targetRPS = 0.;

    private ComponentPublisher m_componentPublisher;
    private double targetAngleDegForSim = 0.0;
    private double currentAngleDegForSim = 0.0;

    public IntakerSubsystem(){
        this.m_componentPublisher = new ComponentPublisher("IntakerPose");
        if(Robot.isReal()){
            io = new IntakerIOPhoenix6();
       } else{
            io = new IntakerIOSim();
        }
    }

    public void setRPS(double rps){
        targetRPS = rps;
        io.setRPS(rps);
    }

    public boolean isAtTargetRPS(){
        return MathUtil.isNear(targetRPS, inputs.intakerVelocityRPS, IntakerConstants.IntakerVelocityToleranceRPS);
    }

    public void stop(){
        setRPS(0.);
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    // FOR SIM
    public void startIntake3D() {  
            System.out.println("Intake started in simulation.");
        targetAngleDegForSim = -90.0;
    }
    public void stopIntake3D() {
            System.out.println("Intake stopped in simulation.");
        targetAngleDegForSim = 0.0;
    }
    public boolean hasCoral() {
        return inputs.hasGamePiece;
    }

    public boolean ejectCoral() {
        if (io instanceof IntakerIOSim sim) {
            return sim.ejectCoral();
        }
        return false; // TODO
    }

    public boolean addCoral() {
        if (io instanceof IntakerIOSim sim) {
            return sim.addCoral();
        }
        return false; // TODO
    }

    @Override
    public void periodic() {
        processLog();
        processDashboard();

        double step = 10.0; // maximum angle change per step (degrees)
        if (Math.abs(targetAngleDegForSim - currentAngleDegForSim) > step) {
            currentAngleDegForSim += Math.signum(targetAngleDegForSim - currentAngleDegForSim) * step;
        } else {
            currentAngleDegForSim = targetAngleDegForSim;
        }
        double angleRad = Math.toRadians(currentAngleDegForSim);
        Pose3d pose = new Pose3d(
            new Translation3d(
                IntakerSimConstants.OFFSET_X,
                IntakerSimConstants.OFFSET_Y,
                IntakerSimConstants.OFFSET_Z
            ),
            new Rotation3d(
                IntakerSimConstants.ROLL,
                angleRad,
                IntakerSimConstants.YAW
            )
        );
        m_componentPublisher.setPose(pose);
    }

    public void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Intaker", inputs);
        Logger.recordOutput("Intaker/TargetRPS", targetRPS);
        Logger.recordOutput("Intaker/VelocityRPS", inputs.intakerVelocityRPS);
    }

    private void processDashboard(){
        SmartDashboard.putNumber("Intaker/TargetRPS", targetRPS);
        SmartDashboard.putNumber("Intaker/VelocityRPS", inputs.intakerVelocityRPS);
    }

}
