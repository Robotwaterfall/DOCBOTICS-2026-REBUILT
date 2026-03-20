package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.subsystems.IntakePitcherSub;

public class FlutterIntake extends SequentialCommandGroup{

    public FlutterIntake(IntakePitcherSub intakePitcherSub){

        addCommands( 
            new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherIn),
            new WaitCommand(Constants.IntakePitcherConstants.intakePitcherWaitTimeSec),
            new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOut)
        );
    }
}
