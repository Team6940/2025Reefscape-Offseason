package frc.robot.commands.CoralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class CoralManualScoring extends Command {
    enum ScoringState {
        AIMING,
        SCORING,
        DEPARTING,
        END
    }

    private int m_targetReefLevelIndex;
    private Button m_executionButton;
    ScoringState state;

    double aimHeight;
    double aimAngle;
    double scoreHeight;
    double scoreAngle;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public CoralManualScoring(int targetReefLevelIndex, Button executionButton) {
        m_targetReefLevelIndex = targetReefLevelIndex;
        m_executionButton = executionButton;
        addRequirements(elevator, shooter, arm, chassis);
    }

    @Override
    public void initialize() {
        state = ScoringState.AIMING;
        aimHeight = Constants.UpperStructureState .valueOf("RPrepareScoreL" + m_targetReefLevelIndex).elevatorHeightMeters;
        aimAngle = Constants.UpperStructureState.valueOf("RPrepareScoreL" + m_targetReefLevelIndex).armAngleDegs;
        scoreHeight = Constants.UpperStructureState.valueOf("RScoreL" + m_targetReefLevelIndex).elevatorHeightMeters;
        scoreAngle = Constants.UpperStructureState.valueOf("RScoreL" + m_targetReefLevelIndex).armAngleDegs;
    }

    @Override
    public void execute() {
        chassis.driveFieldCentric(driverController, 0.3, 0.3);//this may needs tuning afterdays
        switch (state) {
            case AIMING:
                elevator.setHeight(aimHeight);
                arm.setPosition(aimAngle);
                if (driverController.getButton(m_executionButton)) {
                    state = ScoringState.SCORING;
                }
                break;
            case SCORING:
                elevator.setHeight(scoreHeight);
                arm.setPosition(scoreAngle);
                if (driverController.getButton(Button.kA)) {
                    state = ScoringState.DEPARTING;
                }
                if (!driverController.getButton(m_executionButton)) {
                    state = ScoringState.AIMING;
                }
                break;
            case DEPARTING:
                shooter.setRPS(20.);
                if (!driverController.getButton(Button.kA)) {
                    state = ScoringState.END;
                }
            case END:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPosition(FieldConstants.ArmStowPosition);
        elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        if (state == ScoringState.END) {
            return true;
        } else
            return false;
    }
}
