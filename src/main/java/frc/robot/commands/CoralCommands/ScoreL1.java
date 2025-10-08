package frc.robot.commands.CoralCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.GeneralConstants.ArmConstants;
import frc.robot.constants.GeneralConstants.ElevatorConstants;
import frc.robot.constants.GeneralConstants.FieldConstants;
import frc.robot.constants.GeneralConstants.ShooterConstants;
import frc.robot.constants.GeneralConstants.UpperStructureState;
import frc.robot.containers.RobotContainer;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController;
import frc.robot.subsystems.Controller.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.SuperStructure.Selection;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem.ShooterState;

public class ScoreL1 extends Command {

   
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ImprovedCommandXboxController driverController = RobotContainer.driverController;
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double driveDeadband = 0.045;
    public static final double rotateDeadband = 0.045;
    public ScoreL1() {
        addRequirements(elevator, shooter, arm);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setHeight(UpperStructureState.RPrepareScoreL1.elevatorHeightMeters);
        arm.setPosition(UpperStructureState.RPrepareScoreL1.armAngleDegs);
        if(driverController.getButton(Button.kLeftTrigger))
            shooter.setRPS(ShooterConstants.CoralScoringRPS);
        else
            shooter.setRPS(0);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            SmartDashboard.putString("CORAL hybrid Scoring State", "END due to interruption");
            arm.setPosition(FieldConstants.ArmStowPosition);
            elevator.setHeight(ElevatorConstants.MinHeight);
        shooter.stop();
    }


}
