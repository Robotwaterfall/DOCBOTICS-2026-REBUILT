package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConveyorConstant;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.ShooterSub;

public class runConveyorCmd extends Command{

    ConveyorSub conveyorSub;
    ShooterSub shooterSub;

    double desiredVelocity;

    public runConveyorCmd(ConveyorSub conveyorSub, ShooterSub shooterSub){

        this.conveyorSub = conveyorSub;
        this.shooterSub = shooterSub;

        desiredVelocity = shooterSub.getDesiredVelocity();
        addRequirements(conveyorSub);
    }

    @Override
    public void initialize(){
        conveyorSub.setConveyorPower(0);

    }

    @Override
    public void execute(){
        if((shooterSub.getShooter_LeadVelocity()) > (desiredVelocity - ShooterConstants.shooterTolerance) &&
                (shooterSub.getShooter_LeadVelocity()) < (desiredVelocity + ShooterConstants.shooterTolerance)){

                    conveyorSub.setConveyorPower(ConveyorConstant.conveyorPower);

                } else{
                    conveyorSub.setConveyorPower(0);
                }

    }


}
