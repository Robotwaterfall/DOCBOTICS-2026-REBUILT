package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSub;

public class IncrementHoodCMD extends InstantCommand {

    private final HoodSub hoodSub;
    private double deltaHoodAngle;

    public IncrementHoodCMD(HoodSub hoodSub){

        this(hoodSub, HoodConstants.defaultHoodAnglePlusPerPress);

    }
    public IncrementHoodCMD(HoodSub hoodSub, double deltaHoodAngle){

        this.hoodSub = hoodSub;
        this.deltaHoodAngle = deltaHoodAngle;
        addRequirements(hoodSub);

    }
    
    @Override
    public void execute(){
        hoodSub.incrementAngle(deltaHoodAngle);
    }

}