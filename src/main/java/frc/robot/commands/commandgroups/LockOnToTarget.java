package frc.robot.commands.commandgroups;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToTargetCMD;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LedSub;

public class LockOnToTarget extends SequentialCommandGroup{

    public LockOnToTarget(SwerveSub swerveSub,LedSub ledSub, Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction){

        addCommands(
            new AlignToTargetCMD(swerveSub, ledSub, xSpdFunction, ySpdFunction)
        );

    }

}
