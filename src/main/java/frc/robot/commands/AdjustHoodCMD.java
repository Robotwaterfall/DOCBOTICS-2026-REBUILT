package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;

public class AdjustHoodCMD extends Command{

    HoodSub hoodSub;
    SwerveSub swerveSub;

    public AdjustHoodCMD(HoodSub hoodSub, SwerveSub swerveSub){
        this.hoodSub = hoodSub;
        this.swerveSub = swerveSub;
        addRequirements(hoodSub);
    }

     @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);
        if(targetDistanceInches <= 0){return;}

        double targetDistanceFeet = targetDistanceInches / 12.0;
        var hoodParams = ShooterLookup.getInterpolated(targetDistanceFeet);

        hoodSub.setHoodAngle(hoodParams.angleDeg);
    }

    @Override
    public boolean isFinished(){
        return hoodSub.isAtSetAngle();
        
    }

}
