package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSub;

public class DecrementShooterCMD extends InstantCommand{

    private final ShooterSub shooterSub;

    public DecrementShooterCMD(ShooterSub shooterSub){

        this(shooterSub, -ShooterConstants.defaultShooterVelocityPlusPerPress);

    }
    public DecrementShooterCMD(ShooterSub shooterSub, double deltaFps){

        this.shooterSub = shooterSub;
        shooterSub.decrementVelocity(deltaFps);
        addRequirements(shooterSub);

    }

}
