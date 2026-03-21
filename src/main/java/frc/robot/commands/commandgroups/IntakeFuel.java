package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.commands.RunIntakeRollersCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IntakeRollersSub;

public class IntakeFuel extends ParallelCommandGroup{

    public IntakeFuel(IntakeRollersSub intakeRollersSub, ConveyorSub conveyorSub){

        addCommands( 
            new RunIntakeRollersCMD(intakeRollersSub, Constants.IntakeRollerConstants.kIntakeMotorPower),
            new RunConveyorCMD(conveyorSub, Constants.ConveyorConstant.conveyorPower)
        );

    }

}
