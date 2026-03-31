package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSub;

/*
 * Consider adding juggling command to help ball intake. This command would be RunConveyorShooting or something.
 * 
 */

public class RunConveyorCMD extends Command{

    ConveyorSub conveyorSub;
    double conveyorPower;

    public RunConveyorCMD(ConveyorSub conveyorSub, double conveyorPower){

        this.conveyorSub = conveyorSub;
        this.conveyorPower = conveyorPower;
       
        addRequirements(conveyorSub);
    }

    @Override
    public void initialize(){
        conveyorSub.setConveyorPower(0);
    }

    @Override
    public void execute(){
        conveyorSub.setConveyorPower(conveyorPower);
    }

    @Override
    public void end(boolean interrupted){
        conveyorSub.setConveyorPower(0);
    }


}