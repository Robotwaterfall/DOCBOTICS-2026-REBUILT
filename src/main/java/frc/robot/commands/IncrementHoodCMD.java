package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSub;

public class IncrementHoodCMD extends InstantCommand {

    private final HoodSub hoodSub;

    public IncrementHoodCMD(HoodSub hoodSub){

        this(hoodSub, HoodConstants.defaultHoodAnglePlusPerPress);

    }
    public IncrementHoodCMD(HoodSub hoodSub, double deltaHoodAngle){

        this.hoodSub = hoodSub;
        hoodSub.incrementAngle(deltaHoodAngle);
        addRequirements(hoodSub);

    }

}
