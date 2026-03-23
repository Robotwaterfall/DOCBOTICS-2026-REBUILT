package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class ResetPoseCMD extends Command{

    SwerveSub swerveSub;

    public ResetPoseCMD(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);

    }

     @Override
    public void execute() {
        swerveSub.resetPose();
    }

}
