package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.ShooterSub;
import frc.robot.util.ShooterLookup;
import frc.robot.util.ShooterPhysics;
import frc.robot.util.ShooterPhysicsDeprecated;

public class RunShooterCMD extends Command {

    private final ShooterSub shooterSub;

    private double desiredVelocity; // fps
    private final boolean distanceBased;

    ShooterPhysics CalculateShot = new ShooterPhysics();
    ShooterPhysicsDeprecated calculateShotDeprecated = new ShooterPhysicsDeprecated();

    // 1‑param constructor
    // → distance-based: looks up velocity from Limelight distance when command starts
    public RunShooterCMD(ShooterSub shooterSub) {
        this.shooterSub = shooterSub;
        this.desiredVelocity = 0;
        this.distanceBased = true;

        addRequirements(shooterSub);
    }

    // 2‑param constructor
    // → get distance to hub, then set desiredVelocity from lookup table
    public RunShooterCMD(ShooterSub shooterSub, double desiredVelocity) {
        this.shooterSub = shooterSub;
        this.desiredVelocity = desiredVelocity;
        this.distanceBased = false;

        addRequirements(shooterSub);
    }

    public void initialize() {
        if (distanceBased) {
            double distanceFeet = getLimelightDistanceFeet();

            ShooterPhysics.ShotResult r = CalculateShot.computeShot(
            Units.inchesToMeters(distanceFeet),
            Units.inchesToMeters(ShooterConstants.kShooterHeightInches),
            Units.inchesToMeters(ShooterConstants.kHeightOfHubInches),
            Units.inchesToMeters(41.7),
            Units.inchesToMeters(47),
            Units.inchesToMeters(ShooterConstants.kWheelDiameterInches / 2.0),
            ShooterConstants.shooterEfficiency,
            ShooterConstants.kMaxShooterRPM,
            HoodConstants.kShooterFixedAngle
            );

            double velocityFps = Units.metersToFeet(r.velocityMPerSec);

            double velocityErrorPercent = Math.abs(velocityFps - ShooterLookup.getInterpolatedVelocity(distanceFeet)) / velocityFps * 100;
            SmartDashboard.putNumber("%ErrorVelocity", velocityErrorPercent);

            if (!r.valid || velocityErrorPercent >= 3) { // If the physics calculation is invalid or significantly different from the lookup table, use it directly
                this.desiredVelocity = velocityFps;
            } else { // Otherwise, use the lookup table value (or some blend of the two)
                this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
            }
        }
    }

        private double getLimelightDistanceFeet() {
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace(LimelightConstants.LimelightFront);
        double distMeters = Math.sqrt(
            targetPose.getX() * targetPose.getX() +
            targetPose.getY() * targetPose.getY() +
            targetPose.getZ() * targetPose.getZ());
        return Units.metersToInches(distMeters) / 12.0;
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
