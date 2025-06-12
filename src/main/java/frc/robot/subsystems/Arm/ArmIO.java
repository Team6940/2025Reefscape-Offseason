package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default public void setVoltage(double voltage) {
    }

    default public void setRPS(double rps) {
        
    }

    default public void setArmDegree(double degree){

    }

    default public void zeroArmPostion(){
        
    }


    @AutoLog
    public class ArmIOInputs {
        public boolean ArmMotorConnected = false;

        public double ArmVoltageVolts;
        public double ArmCurrentAmps;
        public double ArmVelocityRPS;

        public double ArmRotationDegrees = 0.0;
    }

    default public void updateInputs(ArmIOInputs inputs) {
    }
}
