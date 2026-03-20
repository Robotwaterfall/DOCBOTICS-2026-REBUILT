package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AdjustHoodCMD;
import frc.robot.commands.AlignToHubCMD;
import frc.robot.commands.RunShooterCMD;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class PrepareShot extends Command {

    private final Command full;

    public PrepareShot(
        ShooterSub shooterSub,
        HoodSub hoodSub,
        SwerveSub swerveSub
    ) {
        Command alignAndAimCMD = Commands.parallel(
            new AlignToHubCMD(swerveSub),
            new RunShooterCMD(shooterSub, swerveSub),
            new AdjustHoodCMD(hoodSub, swerveSub)
        );

        Command warmUpFallBackCMD = Commands.parallel(
            new RunShooterCMD(shooterSub, swerveSub, ShooterConstants.kWarmupVelocityFPS),
            new AdjustHoodCMD(hoodSub, swerveSub, HoodConstants.kHoodWarmUpDeg)
        );

        full = Commands.either(
            alignAndAimCMD,
            warmUpFallBackCMD,
            () -> PoseManager.isInAllianceZone(swerveSub)
        );

        addRequirements(shooterSub, hoodSub, swerveSub);
    }

    @Override
    public void initialize() {
        full.initialize();
    }

    @Override
    public void execute() {
        full.execute();
    }

    @Override
    public boolean isFinished() {
        return full.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        full.end(interrupted);
    }
}
