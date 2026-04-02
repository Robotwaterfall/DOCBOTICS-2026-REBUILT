package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSub;

public class ResetHeadingCMD extends InstantCommand  {
    private final SwerveSub swerveSub;

    public ResetHeadingCMD(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
         
    }
    @Override
    public void initialize() {
        //reset heading
        swerveSub.zeroHeading();
    }


}
