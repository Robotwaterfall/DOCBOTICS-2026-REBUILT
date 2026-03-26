package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AdjustHoodCMD;
import frc.robot.commands.RunIndexerCMD;
import frc.robot.commands.RunShooterCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakeRollersSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

public class Juggle extends ParallelCommandGroup{

    public Juggle(ShooterSub shooterSub, IndexerSub indexerSub, SwerveSub swerveSub,
        IntakeRollersSub intakeRollersSub, ConveyorSub conveyorSub, HoodSub hoodSub){

        addCommands(
            new IntakeFuel(intakeRollersSub, conveyorSub)
              
        );

    }

}