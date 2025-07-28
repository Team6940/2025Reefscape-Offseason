package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoGenerator{

    /*This method generates the autonomous command based on the user's choice.
     * It takes an integer (firstChoice) that corresponds to the user's selection and returns the appropriate command.
    */
    
    public static Command generate(int firstChoice){
        switch(firstChoice){
            case 1:
                return new Up4Corals();
            case 2:
                return new Mid2Algae();
            case 3:
                return new Down4Corals();
            case 4:
                return new LeftSide();
            default:
                break;
        }
        return new WaitCommand(0); //TODO What is this for?
    }
}