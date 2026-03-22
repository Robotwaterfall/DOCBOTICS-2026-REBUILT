package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSub;

public class IncrementShooterCMD extends InstantCommand{

    private final ShooterSub shooterSub;

    public IncrementShooterCMD(ShooterSub shooterSub){

        this(shooterSub, ShooterConstants.defaultShooterVelocityPlusPerPress);

    }
    public IncrementShooterCMD(ShooterSub shooterSub, double deltaFps){

        this.shooterSub = shooterSub;
        shooterSub.incrementVelocity(deltaFps);
        addRequirements(shooterSub);

    }

}
