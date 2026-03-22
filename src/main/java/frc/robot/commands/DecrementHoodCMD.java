package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSub;

public class DecrementHoodCMD extends InstantCommand{

    private final HoodSub hoodSub;

    public DecrementHoodCMD(HoodSub hoodSub){

        this(hoodSub, HoodConstants.defaultHoodAnglePlusPerPress);
    }
    public DecrementHoodCMD(HoodSub hoodSub, double deltaHoodAngle){

        this.hoodSub = hoodSub;
        hoodSub.decrementAngle(deltaHoodAngle);
        addRequirements(hoodSub);

    }

}
