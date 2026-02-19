package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSub;

public class ResetHeadingCmd extends Command  {
    private final SwerveSub swerveSub;

    public ResetHeadingCmd(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
         
    }
    @Override
    public void execute() {
        swerveSub.zeroHeading();
    }


}