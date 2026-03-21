package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.commands.RunIndexerCMD;
import frc.robot.commands.RunIntakeRollersCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakeRollersSub;

public class OuttakeFuel extends ParallelCommandGroup{

    public OuttakeFuel(IntakeRollersSub intakeRollersSub, 
        ConveyorSub conveyorSub, IndexerSub indexerSub){

            addCommands(
                new RunIntakeRollersCMD(intakeRollersSub, Constants.IntakeRollerConstants.kOutakeMotorPower),
                new RunIndexerCMD(indexerSub, Constants.ShooterConstants.kReverseIndexSpeed),
                new RunConveyorCMD(conveyorSub, Constants.ConveyorConstant.reverseConveyorPower)
            );

    }

}
