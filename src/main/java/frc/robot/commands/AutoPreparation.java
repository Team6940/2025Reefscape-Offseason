package frc.robot.commands;

import java.net.http.HttpClient.Redirect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class AutoPreparation extends Command {
    enum PreparingState {
        GRABBING,
        RETRACTING,
        END
    }

    private PreparingState state;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();

    @Override
    public void initialize() {
        state = PreparingState.GRABBING;
    }

    @Override
    public void execute() {
        switch (state) {
            case GRABBING:
                grab();
                break;
            case RETRACTING:
                retract();
                break;
            case END:
                break;
        }
    }

    public void grab() {
        elevator.setHeight(ElevatorConstants.IdleHeight);
        arm.setPosition(FieldConstants.ArmStowPosition);
        if (arm.isAtSecuredPosition()) {
            state = PreparingState.RETRACTING;
        }
    }

    public void retract() {
        elevator.setHeight(ElevatorConstants.MinHeight);
        state = PreparingState.END;
    }

    @Override
    public boolean isFinished() {
        if (state == PreparingState.END) {
            return true;
        } else
            return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
