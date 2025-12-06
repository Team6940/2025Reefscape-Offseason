package frc.robot.commands.GroundIntakeCommands;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.Robot;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.constants.GeneralConstants.ShooterConstants;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem.IndexerState;
import frc.robot.subsystems.Intaker.IntakerSubsystem;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class NewCoralAlignSequence extends Command {
    enum IntakeState {
        ALIGNING,
        DROPPING,
        GRABBING,
        RETRACTING,
        END
    }

    private IntakeState state;
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

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
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();

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
        if (!driverController.getButton(Button.kRightTrigger)) {
            indexer.setLeftRPS(-4.);
            indexer.setRghtRPS(-8.);
        } else{
            indexer.setLeftRPS(4.);
            indexer.setRghtRPS(15.); 
            Commands.runOnce(() -> intaker.ejectCoral()); // Eject coral if right trigger is held        
//             if( Robot.isSimulation()){
//                 Commands.runOnce(()->{
//                     SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(
//                     new Pose2d(CommandSwerveDrivetrain.getInstance().getPose().getX()+1., 
//                     CommandSwerveDrivetrain.getInstance().getPose().getY()+1.,
//                     CommandSwerveDrivetrain.getInstance().getPose().getRotation())));
//                 });
//  //TODO
//             }
        }
        // Simulation mode: Once IntakeSimulation is activated, it "collects on contact". As soon as hasCoral() is true, it directly enters GRABBING.
        //TODO delete this when elevator sim is ready.
        if (Robot.isSimulation() && intaker.hasCoral()) {
            state = IntakeState.GRABBING;
        }
        if ((indexer.getIndexerState() == IndexerState.READY || driverController.getButton(m_toggleButton))) {
            state = IntakeState.DROPPING;
        }
    }

    private void drop() {
        elevator.setHeight(ElevatorConstants.GrabbingHeight);
        if (shooter.isShooterReady()) {
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
        // intaker.ejectCoral(); //SIMULATION: immediately transfer coral to indexer
    }

    private void retract() {
        elevator.setHeight(ElevatorConstants.MinHeight);
        state = IntakeState.END;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

}