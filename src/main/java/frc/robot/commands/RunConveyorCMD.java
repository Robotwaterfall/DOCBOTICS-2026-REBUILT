package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstant;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.ShooterSub;

/*
 * Consider adding juggling command to help ball intake. This command would be RunConveyorShooting or something.
 * 
 */

public class RunConveyorCMD extends Command{

    ConveyorSub conveyorSub;
    ShooterSub shooterSub;

    double desiredVelocity;

    private Supplier<Boolean> isReversed;

    public RunConveyorCMD(ConveyorSub conveyorSub, ShooterSub shooterSub, Supplier<Boolean> isReversed){

        this.conveyorSub = conveyorSub;
        this.shooterSub = shooterSub;
        this.isReversed = isReversed;

        desiredVelocity = shooterSub.getDesiredVelocity();
        addRequirements(conveyorSub);
    }

    @Override
    public void initialize(){
        //when command starts set conveyor power to 0
        conveyorSub.setConveyorPower(0);

    }

    @Override
    public void execute(){
        //if the shooter gets to desired velocity start the conveyor for shooting
        if((shooterSub.getShooterLeadVelocity()) > (desiredVelocity - ShooterConstants.shooterTolerance) &&
                (shooterSub.getShooterLeadVelocity()) < (desiredVelocity + ShooterConstants.shooterTolerance)){

                    conveyorSub.setConveyorPower(ConveyorConstant.conveyorPower);

                } else if(isReversed.get()){
                    conveyorSub.setConveyorPower(-ConveyorConstant.conveyorPower);
                } else{
                    conveyorSub.setConveyorPower(0);
                }

    }


}
