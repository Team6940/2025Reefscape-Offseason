package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    default public void setHeight(double targetHeight){

    }

    default public void resetHeight(double _height){

    }

    default public void zeroHeight() {
        resetHeight(0.);
    }

    default public void setVoltage(double voltage){
            
    }

    @AutoLog
    public class ElevatorIOInputs {
        public boolean leftConnected = false;
        public boolean rghtConnected = false;
        public double leftMotorPositionRotations = 0;
        public double rghtMotorPositionRotations = 0;
        public double leftMotorVelocityRPS = 0;
        public double rghtMotorVelocityRPS = 0;

        public double leftVoltageVolts = 0;
        public double rghtVoltageVolts = 0;
        public double leftCurrentAmps = 0;
        public double rghtCurrentAmps = 0;

        public double ElevatorHeight;
        public double ElevatorVelocity;

    }
    default public void updateInputs(ElevatorIOInputs inputs){}
}
