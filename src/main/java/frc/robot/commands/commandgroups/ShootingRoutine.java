package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunShooterCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

public class ShootingRoutine extends SequentialCommandGroup{

    public ShootingRoutine(ShooterSub shooterSub, IndexerSub indexerSub, ConveyorSub conveyorSub, 
        IntakePitcherSub intakePitcherSub, SwerveSub swerveSub){

        addCommands(
            new RunShooterCMD(shooterSub, swerveSub),
            new SequentialCommandGroup(
            new WaitForShooterReady(shooterSub, swerveSub),
            new FireShot(indexerSub, conveyorSub, intakePitcherSub)
            // Lock wheels when firing shot
            )
        );
    }

}
