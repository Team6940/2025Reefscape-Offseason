package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.*;
import frc.robot.Library.MUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase{
    public static ElevatorSubsystem m_Instance = null;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double targetHeight;

    public static ElevatorSubsystem getInstance() {
        return m_Instance == null ? m_Instance = new ElevatorSubsystem() : m_Instance;
    }

    ElevatorSubsystem(){
        if(Robot.isReal()){
            io = new ElevatorIOPhoenix6();
        }
        else {
            //TODO implement simulation
            io = new ElevatorIOPhoenix6();
        }
    };

    public void setHeight(double _Height){
        _Height = MUtils.numberLimit(0, ElevatorConstants.MaxHeight, _Height);
        targetHeight = _Height;
        io.setHeight(targetHeight);
    }

    public void liftHeight(double _Height){
        targetHeight = MUtils.numberLimit(0, ElevatorConstants.MaxHeight, inputs.ElevatorHeight + _Height);
        io.setHeight(targetHeight);
    }

    public double getHeight(){
        return inputs.ElevatorHeight;
    }

    public double getVelocity(){
        return inputs.ElevatorVelocity;
    }

    public double getTargetHeight(){
        return targetHeight;
    }

   public boolean isAtTargetHeight(){
       return MathUtil.isNear(targetHeight, inputs.ElevatorHeight, ElevatorConstants.ElevatorHeightTolerence);
   }

    public void resetHeight(double _height){
        io.resetHeight(_height);
    }

    public void zeroHeight(){
        io.zeroHeight();
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }

    public void processLog(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator/TargetHeight", targetHeight);
    }

    public void processDashboard(){
        SmartDashboard.putNumber("Elevator/Height", inputs.ElevatorHeight);
        SmartDashboard.putNumber("Elevator/Rotation", inputs.leftMotorPositionRotations);
    }

    @Override
    public void periodic(){
        processLog();
        processDashboard();
    }
}
