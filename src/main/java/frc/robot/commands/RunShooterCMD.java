package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;

public class RunShooterCMD extends Command{
    
    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;

    private double targetDistanceInches;

    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub){
        this.shooterSub = shooterSub;
        this.swerveSub = swerveSub;

        this.targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);

        addRequirements(shooterSub);
    }

    @Override
    public void initialize(){
        shooterSub.setShooterVelocityFPS(0);
    }

    @Override
    public void execute(){
        double distanceFeet = targetDistanceInches / 12.0;

        ShooterLookup.ShooterParams params = ShooterLookup.getInterpolated(distanceFeet);

        shooterSub.setShooterVelocityFPS(params.velocityFps);

    }

    @Override
    public boolean isFinished(){
        return shooterSub.isAtSetVelocityFPS();
        
    }

    @Override
    public void end(boolean interrupted){
        shooterSub.setShooterVelocityFPS(0);
    }

}
