package frc.robot.subsystems.GroundIntaker.G_Arm;

import org.littletonrobotics.junction.AutoLog;

public interface G_ArmIO {
    default public void setVoltage(double voltage){}
    default public void setRPS(double rps){}

    @AutoLog
    public class ArmIOInputs {
        public boolean ArmMotorConnected;

        public double ArmVoltageVolts;
        public double ArmCurrentAmps;
        public double ArmVelocityRPS;
    }

    default public void updateInputs(ArmIOInputs inputs){}
}
