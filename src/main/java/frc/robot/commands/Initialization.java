package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;

public class Initialization extends Command {
    // This command is used to initialize everything in the robot when sth is wrong.
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    ClimberSubsystem climber = ClimberSubsystem.getInstance();
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    private Button m_toggleButton;
    private ImprovedCommandXboxController driverController = RobotContainer.driverController;

    public Initialization(Button toggleButton) {
        addRequirements(arm, chassis, climber, elevator, grArm, indexer, intaker, shooter);
        m_toggleButton = toggleButton;
    }

    @Override
    public void initialize() {
        chassis.brake();
        climber.setPosition(ClimberConstants.ClimberDefaultPos);
        grArm.setPosition(GrArmConstants.ExtendedPosition);

        elevator.setHeight(ElevatorConstants.DroppingHeight);
        arm.setPosition(ArmConstants.DroppingPositionDegs);
        shooter.setRPS(ShooterConstants.DroppingRPS);

        intaker.setRPS(IntakerConstants.ReversingRPS); // push the coral out of the intaker
        indexer.setRPS(IndexerConstants.ReversingRPS); // push the coral out of the indexer
    }

    @Override
    public void execute() {
        if (!driverController.getButton(m_toggleButton)){
            shooter.stop();
            arm.reset();;//TODO
            elevator.setHeight(ElevatorConstants.IdleHeight);

            intaker.setRPS(0);
            indexer.setRPS(0);
            grArm.setPosition(GrArmConstants.RetractedPosition);
        }        
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setHeight(ElevatorConstants.IdleHeight);
        arm.reset();
        shooter.setRPS(0);
        grArm.setPosition(Constants.GrArmConstants.RetractedPosition);
        intaker.setRPS(0);
        indexer.setRPS(0);
    }
}
