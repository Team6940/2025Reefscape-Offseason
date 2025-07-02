package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Arm.ArmSubsystem;
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

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();

    public CoralAlignSequence() {
        addRequirements(shooter, elevator, arm, indexer);
    }

    @Override
    public void initialize() {
        state = IntakeState.ALIGNING;
        arm.setPosition(0); // Adjust as necessary for your arm's initial position
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
        if (indexer.getIndexerState() == IndexerState.FREE_SPINNING) {
            state = IntakeState.GRABBING;
        }
    }

    private void grab() {
        elevator.zeroHeight();
        if (shooter.getShooterState() == ShooterState.READY) {
            state = IntakeState.END;
            shooter.setRPS(ShooterConstants.HoldingCoralRPS); //get hold of the coral in case the robot throws it out accidently
        }

    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        elevator.setHeight(0);
        arm.reset();
        indexer.stop();
    }

}