package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;
import frc.robot.util.ShooterPhysics;

public class RunShooterCMD extends Command {

    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;

    private double desiredVelocity; // fps
    ShooterPhysics CalculateShot = new ShooterPhysics();

    // 2‑param constructor
    // → get distance to hub, then set desiredVelocity from lookup table
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub) {
        this.shooterSub = shooterSub;
        this.swerveSub  = swerveSub;

        double targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);
        double distanceFeet = targetDistanceInches / 12.0;

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

        ShooterLookup.ShooterParams params = ShooterLookup.getInterpolated(distanceFeet);
        double velocityFps = Units.metersToFeet(r.velocityMPerSec);

        double velocityErrorPercent = Math.abs(params.velocityFps - velocityFps) / velocityFps * 100;
        SmartDashboard.putNumber("%ErrorVelocity", velocityErrorPercent);

        if (!r.valid) { // If the physics calculation is invalid, fall back to the lookup table value
            this.desiredVelocity = params.velocityFps;
        } else {
            if (velocityErrorPercent >= 3) { // If the physics calculation is significantly different from the lookup table, use it directly
                this.desiredVelocity = Math.toDegrees(r.angleRad);
            } else { 
                this.desiredVelocity = (velocityFps - params.velocityFps) * 0.5; // Average the two values if they are close enough
            }
        }
        addRequirements(shooterSub);
    }

    // 3‑param constructor
    // → desiredVelocity is passed explicitly
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub, double desiredVelocity) {
        this.shooterSub    = shooterSub;
        this.swerveSub     = swerveSub;
        this.desiredVelocity = desiredVelocity;

        addRequirements(shooterSub);
    }

    @Override
    public void execute() {
        shooterSub.setShooterVelocityFPS(desiredVelocity);

        SmartDashboard.putNumber("DesiredVelocity", desiredVelocity);
    }

    @Override
    public boolean isFinished() {
        // return shooterSub.isAtSetVelocityFPS();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stopShooterMotors();
    }
}
