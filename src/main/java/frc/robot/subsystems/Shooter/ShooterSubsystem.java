package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem m_Instance;

    public static ShooterSubsystem getInstance() {
        return m_Instance == null ? m_Instance = new ShooterSubsystem() : m_Instance;
    }

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final LinearFilter currentFilter = LinearFilter.movingAverage(ShooterConstants.CurrentFilterTaps);
    private final Debouncer currentDebouncer = new Debouncer(ShooterConstants.ShooterDebouncerTime,
            DebounceType.kRising);// detects the current rapid risings

    private double targetRPS = 0;

    public ShooterState state = ShooterState.IDLE;

    public ShooterSubsystem() {
        if (Robot.isReal()) {
            io = new ShooterIOPhoenix6();
            // io = new ShooterIOEmpty();
        } else {
            // TODO: Implement simulation code here
            io = new ShooterIO(){};
        }
    }

    public void setRPS(double rps) {
        targetRPS = rps;
        io.setRPS(rps);
        state = ShooterState.DUTY;
    }

    public boolean IsAtTargetRps() {
        return MathUtil.isNear(targetRPS, inputs.shooterVelocityRPS, ShooterConstants.ShooterSpeedTolerence);
    }

    public void stop() {
        targetRPS = 0;
        io.setRPS(0);
    }

    public void setVoltage(double voltage) {
        targetRPS = 0;
        io.setVoltage(voltage);
    }

    public enum ShooterState {
        IDLE,
        DUTY,
        READY
    }

    public ShooterState getShooterState() {
        return state;
    }

    public boolean isShooterReady() {
        return state == ShooterState.READY;
    }

    public void shooterStateUpdate() {
        double rawCurrent = inputs.motorCurrentAmps;
        double filteredCurrent = currentFilter.calculate(rawCurrent);// get the filtered current


        boolean rising = filteredCurrent > ShooterConstants.ShooterIntakeCurrentThreshold;
        // boolean rising = rawCurrent > ShooterConstants.ShooterIntakeCurrentThreshold;
        boolean spikeDetected = currentDebouncer.calculate(rising);

        // state = ShooterState.IDLE;

        // if (rising || spikeDetected) {
        if (rising) {
            state = ShooterState.READY;
        } 
        else {
            state = ShooterState.IDLE;
        }
    }

    public void resetShooterState() {
        state = ShooterState.IDLE;
        currentDebouncer.calculate(false); // Reset debouncer
    }
    public double getAmp()
    {
        return currentFilter.calculate(inputs.motorCurrentAmps);
    }

    @Override
    public void periodic() {
        // if (state == ShooterState.DUTY || state == ShooterState.READY) {
        //     shooterStateUpdate();
        // }
        shooterStateUpdate();
        processLog();
        processDashboard();

        SmartDashboard.putString("status", getShooterState().toString());
    }

    private void processLog() {
        io.updateInputs(inputs);
        inputs.state= getShooterState();
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/TargetRPS", targetRPS);
        Logger.recordOutput("Shooter/IsAtTargetRPS", IsAtTargetRps());
        Logger.recordOutput("Shooter/isShooterReady",isShooterReady());
    }

    private void processDashboard() {
        // TODO: Implement dashboard code here

    }

    public double getShootRPS() {
        return inputs.shooterVelocityRPS;
    }

}
