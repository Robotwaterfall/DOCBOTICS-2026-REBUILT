package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.commands.MoveIntakePitcherCMD;
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

            Commands.repeatingSequence(
                new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.kPitcherOutDegrees),
                new WaitCommand(IntakePitcherConstants.intakePitcherWaitTimeSec),
                new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.intakePitcherFlutterDegrees),
                new WaitCommand(IntakePitcherConstants.intakePitcherWaitTimeSec)
            )
        );
      
    }

}
