package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSub;

public class DecrementHoodCMD extends Command{

    private final HoodSub hoodSub;

    public DecrementHoodCMD(HoodSub hoodSub){

        this(hoodSub, HoodConstants.defaultHoodAnglePlusPerPress);

    }
    public DecrementHoodCMD(HoodSub hoodSub, double deltaHoodAngle){

        this.hoodSub = hoodSub;
        hoodSub.incrementAngle(deltaHoodAngle);
        addRequirements(hoodSub);

    }

}
