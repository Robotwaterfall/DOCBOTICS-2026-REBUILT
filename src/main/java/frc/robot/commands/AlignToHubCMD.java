package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.SwerveSub;

public class AlignToHubCMD extends Command{

    LimelightSub llSub;
    SwerveSub swerveSub;

    public AlignToHubCMD(LimelightSub llSub, SwerveSub swerveSub){
        this.llSub = llSub;
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        
    }

    @Override
    public boolean isFinished(){
        return false;
        
    }

}
