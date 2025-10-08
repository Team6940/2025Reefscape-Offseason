package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.IndexerConstants;
import frc.robot.constants.GeneralConstants.ShooterConstants;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem.IndexerState;

import edu.wpi.first.wpilibj2.command.Command;

public class CoralAlignSequence extends Command {
    enum IntakeState {
        ALIGNING,
        GRABBING,
        END
    }

    private IntakeState state;
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

    /**This works as an emergency continue button, in case the sensor is <b>not</b> working properly. */
    private Button m_toggleButton; 

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();

    public CoralAlignSequence(Button toggleButton) {
        addRequirements(shooter, elevator, arm, indexer);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        elevator.setHeight(ElevatorConstants.IdleHeight);
        state = IntakeState.ALIGNING;
        arm.reset(); // Adjust as necessary for your arm's initial position
        shooter.setRPS(0);
    }

    @Override
    public void execute() {
        switch (state) {
            case ALIGNING:
                align();
                break;
            case GRABBING:
                grab();
                break;
            case END:
                break;
        }
    }

    private void align() {
        shooter.setRPS(ShooterConstants.CoralIntakingRPS);
        elevator.setHeight(ElevatorConstants.IntakingHeight);
        indexer.setRPS(IndexerConstants.IntakingRPS);
        if (indexer.getIndexerState() == IndexerState.READY || driverController.getButton(m_toggleButton)) {
            state = IntakeState.GRABBING;
        }
        // if (indexer.getIndexerState() == IndexerState.FREE_SPINNING) {
        //     state = IntakeState.GRABBING;
        // }
    }

    private void grab() {
        arm.reset();
        elevator.setHeight(ElevatorConstants.GrabbingHeight);
        if (shooter.getShooterState() == ShooterState.READY) {
            shooter.setRPS(ShooterConstants.HoldingCoralRPS); //get hold of the coral in case the robot throws it out accidently
            elevator.setHeight(0);//TODO
            state = IntakeState.END;
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        elevator.setHeight(ElevatorConstants.IdleHeight);//TODO : ALL ELEVATOR SET HEIGHT 0 SHOULD BE CHANGED TO SET HEIGHT IDLE_HEIGHT, ENSURING NO CONFLICTING WITH INDEXER
        arm.reset();
        indexer.stop();
    }

}