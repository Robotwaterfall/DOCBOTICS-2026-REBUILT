package frc.robot.commands.commandgroups;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToTargetCMD;
import frc.robot.commands.LockWheelsCMD;
import frc.robot.subsystems.SwerveSub;

public class LockOnToTarget extends SequentialCommandGroup{

    public LockOnToTarget(SwerveSub swerveSub, Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction){

        addCommands(
            new AlignToTargetCMD(swerveSub, xSpdFunction, ySpdFunction)
        );

    }

}
