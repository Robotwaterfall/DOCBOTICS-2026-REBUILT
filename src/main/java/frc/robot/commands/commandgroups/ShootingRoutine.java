package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooterCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.ShooterSub;

public class ShootingRoutine extends SequentialCommandGroup{

    public ShootingRoutine(ShooterSub shooterSub, IndexerSub indexerSub, ConveyorSub conveyorSub, IntakePitcherSub intakePitcherSub){

        addCommands(
            new RunShooterCMD(shooterSub),
            new SequentialCommandGroup(
            new WaitForShooterReady(shooterSub),
            new FireShot(indexerSub, conveyorSub, intakePitcherSub)
            )
        );
    }

}
