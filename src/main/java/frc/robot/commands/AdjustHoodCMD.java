package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSub;

public class AdjustHoodCMD extends InstantCommand {

    private final HoodSub hoodSub;
    private final double angleDeg;

    public AdjustHoodCMD(HoodSub hoodSub, double angleDeg) {
        this.hoodSub = hoodSub;
        this.angleDeg = angleDeg;
        addRequirements(hoodSub);
    }

    @Override
    public void execute() {
        hoodSub.setHoodAngle(angleDeg);
    }
}
