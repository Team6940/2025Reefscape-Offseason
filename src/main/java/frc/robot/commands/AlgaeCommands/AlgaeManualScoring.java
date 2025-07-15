package frc.robot.commands.AlgaeCommands;


import java.lang.reflect.Field;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

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
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SuperStructure.RobotStatus;

public class AlgaeManualScoring extends Command{
 
    private double m_targetHeight;
    private double m_targetAngle;
    private Button m_executionButton;

    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;
    SuperStructure superStructure = SuperStructure.getInstance();


    public AlgaeManualScoring(Button executionButton){
        addRequirements(elevator,shooter,arm);
        m_targetAngle = FieldConstants.ArmStowPosition;
        m_targetHeight = FieldConstants.ElevatorAlgaeScoreHeight;
    }

    @Override
    public void initialize(){
        elevator.setHeight(m_targetHeight);
        arm.setPosition(m_targetAngle);
        
    }

    @Override
    public void execute() {
        if(driverController.getButton(m_executionButton)){
            score();
        }
    }

    private void score(){
        shooter.setRPS(ShooterConstants.AlgaeScoringRPS);
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPosition(FieldConstants.ArmStowPosition);
        elevator.setHeight(ElevatorConstants.MinHeight);
    }
}


