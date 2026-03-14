package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class DumpShooterCmd extends Command{

    ShooterSub shooterSub;

    public DumpShooterCmd(ShooterSub shooterSub) {
        this.shooterSub = shooterSub;

        addRequirements(shooterSub);
    }

    public void initialize(){
        
         shooterSub.setShooterVelocityMPS(0);

    }

    public void execute(){

        shooterSub.setShooterVelocityMPS(-0.5);

    }

    public boolean isFinished(){
        return false;
    }
    
    public void end(boolean interrupted){
        shooterSub.setShooterVelocityMPS(0);
    }

}
