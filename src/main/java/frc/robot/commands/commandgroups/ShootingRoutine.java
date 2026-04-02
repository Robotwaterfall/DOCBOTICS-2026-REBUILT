package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooterCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.ShooterSub;

public class ShootingRoutine extends SequentialCommandGroup{

    public ShootingRoutine(ShooterSub shooterSub, IndexerSub indexerSub, ConveyorSub conveyorSub){

        addCommands(
            new RunShooterCMD(shooterSub),
            new WaitForShooterReady(shooterSub),
            new FireShot(indexerSub, conveyorSub)
        );
    }

}
