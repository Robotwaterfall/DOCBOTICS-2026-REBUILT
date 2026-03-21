package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToHubCMD;
import frc.robot.subsystems.ShooterSub;

public class WaitForShooterReady extends Command{
    private final ShooterSub shooterSub;
    private final AlignToHubCMD alignToHubCMD;

    public WaitForShooterReady(ShooterSub shooterSub, AlignToHubCMD alignToHubCMD){
        this.shooterSub = shooterSub;
        this.alignToHubCMD = alignToHubCMD;
        addRequirements(shooterSub);
    }

    @Override
    public boolean isFinished() {
        return shooterSub.isAtSetVelocityFPS() && alignToHubCMD.isFinished();
    }

}
