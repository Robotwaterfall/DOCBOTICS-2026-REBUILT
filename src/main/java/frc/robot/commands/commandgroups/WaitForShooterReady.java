package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class WaitForShooterReady extends Command{
    private final ShooterSub shooterSub;

    public WaitForShooterReady(ShooterSub shooterSub){
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override
    public boolean isFinished() {
        return shooterSub.isAtSetVelocityFPS();
    }

}