package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSub;

public class IncrementShooterCMD extends InstantCommand{

    private final ShooterSub shooterSub;
    private double deltaShooterVelocity;

    public IncrementShooterCMD(ShooterSub shooterSub){

        this(shooterSub, Constants.ShooterConstants.defaultShooterVelocityPlusPerPress);

    }
    public IncrementShooterCMD(ShooterSub shooterSub, double deltaShooterVelocity){

        this.shooterSub = shooterSub;
        this.deltaShooterVelocity = deltaShooterVelocity;
        addRequirements(shooterSub);

    }
    
    @Override
    public void execute(){
        shooterSub.incrementVelocity(deltaShooterVelocity);
    }

}