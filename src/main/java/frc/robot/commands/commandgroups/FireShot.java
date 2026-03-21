package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.commands.RunIndexerCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;

public class FireShot extends ParallelCommandGroup{

    public FireShot(IndexerSub indexerSub, ConveyorSub conveyorSub, IntakePitcherSub intakePitcherSub){
        addCommands(
            new RunIndexerCMD(indexerSub, Constants.ShooterConstants.kIndexSpeed),
            new RunConveyorCMD(conveyorSub, Constants.ConveyorConstant.conveyorPower),
            new FlutterIntake(intakePitcherSub).repeatedly()
        );
      
    }

}
