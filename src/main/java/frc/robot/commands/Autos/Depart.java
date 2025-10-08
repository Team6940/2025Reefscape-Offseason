package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.containers.RobotContainer;

public class Depart extends SequentialCommandGroup{

    /** generates a departure Command
     * @param startP1 the starting barge, 1 for U, 2 for D
     * @param startP2 the starting pose in barge, 1 for U, 2 for M, 3 for D
     */
    public Depart(int startP1, int startP2){
        String pathName, mod1 = "", mod2 = "";
        switch (startP1) {          //TODO: if departing from other side is possible then keep this
            case 1:
                mod1 = "U";
                break;
            default:
                DriverStation.reportWarning("Invalid startP1", true);
                return;
        }

        switch (startP2){
            case 1:
                mod2 = "U";
                break;
            case 2:
                mod2 = "M";
                break;
            case 3:
                mod2 = "D";
                break; 
            default:
                DriverStation.reportWarning("Invalid startP1", true);
                return;
        }
        pathName = mod1 + "B" + mod2 + "_D"; //TODO
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            addCommands(
                new InstantCommand(() ->
                    RobotContainer.chassis.resetPose(
                        RobotContainer.chassis.generateChoreoPath(pathName).getStartingHolonomicPose().get()
                    )
                )
            );
        } else {
            addCommands(
                new InstantCommand(() ->
                    RobotContainer.chassis.resetPose(
                        RobotContainer.chassis.generateChoreoPath(pathName).flipPath().getStartingHolonomicPose().get()
                    )
                )
            );
        }

        addCommands(RobotContainer.chassis.followChoreoPath(pathName));

    }
}
