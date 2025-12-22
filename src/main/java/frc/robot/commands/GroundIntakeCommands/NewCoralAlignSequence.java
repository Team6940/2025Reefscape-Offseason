package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.SuperStructure;
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
        DROPPING,
        GRABBING,
        RETRACTING,
        END
    }

    private boolean isIntakingFromRghtSide = true;
    private IntakeState state;
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;
    private ImprovedCommandXboxController operatorController = RobotContainer.operatorController;

    /**
     * This works as an emergency continue button, in case the sensor is <b>not</b>
     * working properly.
     */
    private Button m_toggleButton;

    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    SuperStructure robot = SuperStructure.getInstance();

    public NewCoralAlignSequence(Button toggleButton) {
        addRequirements(shooter, elevator, arm, indexer);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        elevator.setHeight(ElevatorConstants.IdleHeight);
        arm.reset(); // Adjust as necessary for your arm's initial position
        shooter.setRPS(0);
        state = IntakeState.ALIGNING;
    }

    @Override
    public void execute() {
        switch (state) {
            case ALIGNING:
                align();
                break;
            case DROPPING:
                drop();
                break;
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

    private void align() {
        shooter.setRPS(ShooterConstants.CoralIntakingRPS);
        elevator.setHeight(ElevatorConstants.IntakingHeight);
        
        if(operatorController.getButton(Button.kLeftTrigger)){
            isIntakingFromRghtSide = false;
        }
        else if(operatorController.getButton(Button.kRightTrigger)){
            isIntakingFromRghtSide = true;
        }

        if(driverController.getButton(Button.kA)){
            indexer.setLeftRPS(15.);//Left CCW
            indexer.setRghtRPS(-15.);//Rght CW
        }//reversing coral out of indexer
        else{
            if(isIntakingFromRghtSide){
                indexer.setLeftRPS(4.);//slightly reversing
                indexer.setRghtRPS(15.);//intake
            }
            else{
                indexer.setLeftRPS(-15.);//intake
                indexer.setRghtRPS(-4.);//slightly reversing
            }
        }

        if ((
                driverController.getButton(m_toggleButton)) && arm.isAtTargetPositon()) {
            // if (driverController.getButton(m_toggleButton)) {

            state = IntakeState.DROPPING;
        }
    }

    private void drop() {
        elevator.setHeight(ElevatorConstants.GrabbingHeight);
        if (!driverController.getButton(m_toggleButton)) {
            state = IntakeState.GRABBING;

        }
    }

    private void grab() {
        indexer.stop();
        shooter.stop();
        elevator.setHeight(ElevatorConstants.IdleHeight);
        arm.setPosition(FieldConstants.ArmStowPosition);
        if (arm.isAtSecuredPosition()) {
            state = IntakeState.RETRACTING;
        }
        // if(arm.isAtSecuredPosition()){
        // elevator.setHeight(0.1);//TODO MOVE INTO CONSTANTS
        // }
        // // elevator.setHeight(0.1);//TODO MOVE INTO CONSTANTS
        // state = IntakeState.END;
    }

    private void retract() {
        elevator.setHeight(ElevatorConstants.MinHeight);
        state = IntakeState.END;
    }
    @Override
    public boolean isFinished()
    {
            return state==IntakeState.END;
        
    }
    @Override
    public void end(boolean interrupted) {
        // shooter.stop();
        // elevator.setHeight(ElevatorConstants.IdleHeight);// TODO : ALL ELEVATOR SET
        // HEIGHT 0 SHOULD BE CHANGED TO SET
        // // HEIGHT IDLE_HEIGHT, ENSURING NO CONFLICTING WITH INDEXER
        // arm.reset();
        indexer.stop();
    }

}