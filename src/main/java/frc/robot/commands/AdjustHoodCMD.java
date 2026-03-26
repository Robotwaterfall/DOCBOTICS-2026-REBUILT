package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;

public class AdjustHoodCMD extends InstantCommand{

    HoodSub hoodSub;
    SwerveSub swerveSub;

    private double desiredAngleDeg;

    // 2‑param constructor
    // → get distance to hub, then set desiredVelocity from lookup table
    public AdjustHoodCMD(HoodSub hoodSub, SwerveSub swerveSub) {
        this.hoodSub = hoodSub;
        this.swerveSub  = swerveSub;

        double targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);
        double distanceFeet         = targetDistanceInches / 12.0;

        ShooterLookup.ShooterParams params = ShooterLookup.getInterpolated(distanceFeet);
        this.desiredAngleDeg = params.angleDeg;

        addRequirements(hoodSub);
    }

    // 3‑param constructor
    // → desiredVelocity is passed explicitly
    public AdjustHoodCMD(HoodSub hoodSub, SwerveSub swerveSub, double desiredAngleDeg) {
        this.hoodSub    = hoodSub;
        this.swerveSub     = swerveSub;
        this.desiredAngleDeg = desiredAngleDeg;

        addRequirements(hoodSub);
    }

     @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        hoodSub.setHoodAngle(desiredAngleDeg);
    }

}
