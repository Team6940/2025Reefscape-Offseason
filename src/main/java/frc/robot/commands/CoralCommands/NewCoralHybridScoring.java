package frc.robot.commands.CoralCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UpperStructureState;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Chassis.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class NewCoralHybridScoring extends Command {
    enum ScoringState {
        AIMING,
        SCORING,
        DEPARTING,
        END
    }

    private int m_targetReefPoseIndex;
    private int m_targetReefLevelIndex;
    private Button m_executionButton;
    ScoringState state;

    Pose2d targetPose;
    double aimHeight;
    double aimAngle;
    double scoreHeight;
    double scoreAngle;
    boolean m_isReversed;
    double targetRotation;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    private final Timer triggerTimer = new Timer();

    public NewCoralHybridScoring(int targetReefPoseIndex, int targetReefLevelIndex, Button executionButton, boolean isReversed) {
        addRequirements(elevator, shooter, chassis, arm);
        m_targetReefPoseIndex = targetReefPoseIndex;
        m_targetReefLevelIndex = targetReefLevelIndex;
        m_executionButton = executionButton;
        m_isReversed=isReversed;
    }

    @Override
    public void initialize() {
        state = ScoringState.AIMING;
        // if(m_targetReefLevelIndex<=1)
        if(m_isReversed)
        {
            aimHeight = Constants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            aimAngle = Constants.UpperStructureState.valueOf("RPrepareScoreL"+m_targetReefLevelIndex).armAngleDegs;
            scoreHeight=Constants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            scoreAngle=Constants.UpperStructureState.valueOf("RScoreL"+m_targetReefLevelIndex).armAngleDegs;
            targetPose = chassis.generateReefPoseReversed(m_targetReefPoseIndex,m_targetReefLevelIndex);
        }
        else{
            aimHeight = Constants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            aimAngle = Constants.UpperStructureState.valueOf("PrepareScoreL"+m_targetReefLevelIndex).armAngleDegs;
            scoreHeight=Constants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).elevatorHeightMeters;
            scoreAngle=Constants.UpperStructureState.valueOf("ScoreL"+m_targetReefLevelIndex).armAngleDegs;
            targetPose = chassis.generateReefPose(m_targetReefPoseIndex,m_targetReefLevelIndex);

        }

    }

    @Override
    public void execute() {
        if(m_targetReefLevelIndex>=2)
            chassis.hybridMoveToPose(targetPose, driverController,
                FieldConstants.reefTranslationAdjustmentRange, FieldConstants.reefRotationAdjustmentRangeDegs);
        SmartDashboard.putString("CORAL hybrid Scoring State", state.toString());
        Logger.recordOutput("CoralScoringTargetPose", targetPose);
        switch (state) {
            case AIMING:
                aim();
                break;
            case SCORING:
                score();
                break;
            case DEPARTING:
                depart();
                break;
            case END:
                break;
        }
    }

    public void aim() {
        elevator.setHeight(aimHeight);
        arm.setPosition(aimAngle);
        if (
            // arm.isAtTargetPositon() && chassis.isAtTargetPose() && elevator.isAtTargetHeight()
            driverController.getButton(m_executionButton)
        ) 
        {
            triggerTimer.reset();
            triggerTimer.start();
            state = ScoringState.SCORING;
        }
    
    }

    public void score() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING");
        if(m_targetReefLevelIndex>=2){
        arm.setPosition(scoreAngle);
        elevator.setHeight(scoreHeight);

        if (!driverController.getButton(m_executionButton)) {
            triggerTimer.stop();
            if(triggerTimer.get() < 0.3){//&& arm is at position
                if(arm.isAtTargetPositon()){
                    SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING complete, moving to DEPARTING");
                    state = ScoringState.DEPARTING;
                    Transform2d retreatTransform2d = new Transform2d(FieldConstants.CoralScoreRetreatDistance, 0,Rotation2d.kZero);
                    Pose2d departPose = targetPose.plus(retreatTransform2d);
                    targetPose = departPose;
                }
            }
            else{
                state = ScoringState.AIMING;
            }
        }

        if (driverController.getButton(Button.kA)) {
            //if (driverController.getButton(Button.kA)) {
            SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING complete, moving to DEPARTING");
            state = ScoringState.DEPARTING;
            Transform2d retreatTransform2d = new Transform2d(FieldConstants.CoralScoreRetreatDistance, 0, Rotation2d.kZero);
            Pose2d departPose = targetPose.plus(retreatTransform2d);
            targetPose = departPose;
        }
        
        // if (arm.isAtTargetPositon() && elevator.isAtTargetHeight() && driverController.getButton(Button.kA)) {
        //     //if (driverController.getButton(Button.kA)) {
        //     SmartDashboard.putString("CORAL hybrid Scoring State", "SCORING complete, moving to DEPARTING");
        //     state = ScoringState.DEPARTING;
        //     Transform2d retreatTransform2d = new Transform2d(FieldConstants.CoralScoreRetreatDistance, 0, Rotation2d.kZero);
        //     Pose2d departPose = targetPose.plus(retreatTransform2d);
        //     targetPose = departPose;
        // }}

        // if(!driverController.getButton(m_executionButton)){
        //     state = ScoringState.AIMING;
        // }
    }
    }

    public void depart() {
        SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING");

        // Calculate retreat position (move backward)
        // Pose2d currentPose = targetPose;
        // Translation2d transformTranslation2d = new Translation2d(FieldConstants.CoralScorePushDistance,
        //         currentPose.getRotation());
        // Pose2d departPose = new Pose2d(currentPose.getTranslation().plus(transformTranslation2d),
        //         currentPose.getRotation());


        // Move to retreat position
        shooter.setRPS(ShooterConstants.CoralScoringRPS);

        // When in position, reset systems and end
        // if (chassis.isAtTargetPose()) {
        //     arm.setPosition(FieldConstants.ArmStowPosition);
        //     elevator.setHeight(ElevatorConstants.MinHeight);
        //     shooter.stop();
        //     state = ScoringState.END;
        //     SmartDashboard.putString("CORAL hybrid Scoring State", "DEPARTING complete, moving to END");
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("CORAL hybrid Scoring State", "END due to interruption");
            arm.setPosition(FieldConstants.ArmStowPosition);
            elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.stop();
    }

    public NewCoralHybridScoring withSelection(Selection selection) {
        switch (selection) {
            case LEFT:
                return new NewCoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 1, m_targetReefLevelIndex,
                        m_executionButton, m_isReversed);
            case RIGHT:
                return new NewCoralHybridScoring((m_targetReefPoseIndex - 1) / 2 * 2 + 2, m_targetReefLevelIndex,
                        m_executionButton, m_isReversed);

            default:
                return null;
        }
    }

}
