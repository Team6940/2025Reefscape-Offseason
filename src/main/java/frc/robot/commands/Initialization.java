package frc.robot.commands;

import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.GrArm.GrArmSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class Initialization extends Command {
    // This command is used to initialize everything in the robot when sth is wrong.
    ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ArmSubsystem arm = ArmSubsystem.getInstance();
    CommandSwerveDrivetrain chassis = CommandSwerveDrivetrain.getInstance();
    IndexerSubsystem indexer = IndexerSubsystem.getInstance();
    GrArmSubsystem grArm = GrArmSubsystem.getInstance();
    IntakerSubsystem intaker = IntakerSubsystem.getInstance();

    public Initialization(ElevatorSubsystem elevator,ShooterSubsystem shooter,ArmSubsystem arm,
    CommandSwerveDrivetrain chassis,IndexerSubsystem indexer, GrArmSubsystem grArm, IntakerSubsystem intaker) {
        this.elevator= elevator;
        this.shooter = shooter;
        this.arm = arm;
        this.chassis = chassis;
        this.indexer = indexer;
        this.grArm = grArm;
        this.intaker = intaker;
        addRequirements(elevator, shooter, arm, chassis, indexer, grArm, intaker);
    }

    @Override
    public void initialize() {
        ShooterSubsystem.ShooterState shooterState = shooter.getShooterState();
        if (shooterState ==ShooterSubsystem.ShooterState.GRABBING) {
            arm.setPosition(FieldConstants.ArmAngles[2]);//just a random position to make sure the arm is in a safe position
            shooter.setRPS(10.);//shoot the coral out
        }
        grArm.setPosition(Constants.GrArmConstants.ExtendedPosition);
        intaker.setRPS(-10.); // push the coral out of the intaker
        indexer.setRPS(-10.); // push the coral out of the indexer
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setHeight(0);
        arm.setPosition(0);
        shooter.setRPS(0);
        grArm.setPosition(Constants.GrArmConstants.RetractedPosition);
        intaker.setRPS(0);
        indexer.setRPS(0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
    
}
