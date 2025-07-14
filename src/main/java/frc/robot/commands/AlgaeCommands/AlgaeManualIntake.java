package frc.robot.commands.AlgaeCommands;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.AlgaeCommands.AlgaeHybridIntake.IntakeState;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class AlgaeManualIntake extends Command {
    private int m_targetReefFaceIndex;
    private double m_targetHeight;
    private double m_targetAngle;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public AlgaeManualIntake(int targetReefFaceIndex, Button executionButton) {
        addRequirements(elevator, shooter, arm);
    }

    @Override
    public void initialize() {
        m_targetReefFaceIndex = chassis.generateAlgaeIntakeIndex();
        m_targetHeight = FieldConstants.ElevatorAlgaeIntakeHeight[m_targetReefFaceIndex];
        m_targetAngle = FieldConstants.ArmIntakePosition[m_targetReefFaceIndex];
        elevator.setHeight(m_targetHeight);
        arm.setPosition(m_targetAngle);
        shooter.setRPS(ShooterConstants.AlgaeIntakingRPS);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setHeight(ElevatorConstants.MinHeight);
        arm.setPosition(FieldConstants.ArmStowPosition);
        //arm.reset();
        shooter.setRPS(-60.);
    }

    public boolean isFinished() {
        return false;
    }
}//This Command needs deferring