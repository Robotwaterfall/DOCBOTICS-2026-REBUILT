package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;
import frc.robot.util.ShooterPhysics;

public class AdjustHoodCMD extends InstantCommand{

    HoodSub hoodSub;
    SwerveSub swerveSub;

    private double desiredAngleDeg;

    ShooterPhysics CalculateShot = new ShooterPhysics();

    // 2‑param constructor
    // → get distance to hub, then set desiredVelocity from lookup table
    public AdjustHoodCMD(HoodSub hoodSub, SwerveSub swerveSub) {
        this.hoodSub = hoodSub;
        this.swerveSub  = swerveSub;

        double targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);
        double distanceFeet = targetDistanceInches / 12;
        

        ShooterPhysics.ShotResult r = CalculateShot.computeShot(
            Units.inchesToMeters(targetDistanceInches),
            Units.inchesToMeters(ShooterConstants.kShooterHeightInches),
            Units.inchesToMeters(ShooterConstants.kHeightOfHubInches),
            Units.inchesToMeters(41.7),
            Units.inchesToMeters(47),
            Units.inchesToMeters(ShooterConstants.kWheelDiameterInches / 2.0),
            ShooterConstants.shooterEfficiency,
            ShooterConstants.kMaxShooterRPM,
            HoodConstants.minHoodAngleDeg,
            HoodConstants.maxHoodAngleDeg
        );

        if (!r.valid) {
            ShooterLookup.ShooterParams params = ShooterLookup.getInterpolated(distanceFeet);
            this.desiredAngleDeg = params.angleDeg;
        } else {
            this.desiredAngleDeg = Math.toDegrees(r.angleRad);
        }

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
