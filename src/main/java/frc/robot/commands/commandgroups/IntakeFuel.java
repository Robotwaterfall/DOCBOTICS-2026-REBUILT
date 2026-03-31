package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.commands.RunIntakeRollersCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.IntakeRollersSub;

public class IntakeFuel extends ParallelCommandGroup{

    public IntakeFuel(IntakeRollersSub intakeRollersSub, ConveyorSub conveyorSub, 
        IndexerSub indexerSub, IntakePitcherSub intakePitcherSub
    ){

        addCommands( 
            new RunIntakeRollersCMD(intakeRollersSub, Constants.IntakeRollerConstants.kIntakeVelocityRPS),
            new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOutDegrees)
        );

    }

}