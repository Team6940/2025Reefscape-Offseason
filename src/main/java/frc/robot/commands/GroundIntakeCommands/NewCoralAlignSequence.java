package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem.IndexerState;
import edu.wpi.first.wpilibj2.command.Command;

public class NewCoralAlignSequence extends Command {
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

    public NewCoralAlignSequence(Button toggleButton) {
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
        indexer.setRghtRPS(-9.);
        indexer.setLeftRPS(-6.);
        // indexer.setRPS(IndexerConstants.IntakingRPS);
        // if (indexer.getIndexerState() == IndexerState.READY || driverController.getButton(m_toggleButton)) {
        if (driverController.getButton(m_toggleButton)) {

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
            shooter.stop(); //TODO decide whether it's necessary to spin the shooter for getting hold of the coral in case the robot throws it out accidently
            elevator.setHeight(ElevatorConstants.IdleHeight);
            arm.setPosition(FieldConstants.ArmStowPosition);
            if(arm.isAtSecuredPosition()){
                elevator.setHeight(-0.27); //TODO MOVE INTO CONSTANTS
            }
            state=IntakeState.END;
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