package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

public class WaitForShooterReady extends Command{
    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;
    private final LedSub ledSub;

    public WaitForShooterReady(ShooterSub shooterSub,SwerveSub swerveSub,LedSub ledSub){
        this.shooterSub = shooterSub;
        this.swerveSub = swerveSub;
        this.ledSub = ledSub;
        addRequirements(shooterSub, ledSub);
    }

    @Override
    public boolean isFinished() {
        return shooterSub.isAtSetVelocityFPS(); 
    }

}
