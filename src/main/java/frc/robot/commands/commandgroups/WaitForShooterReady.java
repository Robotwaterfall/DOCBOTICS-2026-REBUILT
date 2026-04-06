package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class WaitForShooterReady extends Command{
    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;

    public WaitForShooterReady(ShooterSub shooterSub,SwerveSub swerveSub){
        this.shooterSub = shooterSub;
        this.swerveSub = swerveSub;
        addRequirements(shooterSub);
    }

    @Override
    public boolean isFinished() {
        return shooterSub.isAtSetVelocityFPS() && PoseManager.isInAllianceZone(swerveSub);
    }

}
