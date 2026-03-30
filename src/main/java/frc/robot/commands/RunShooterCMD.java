package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.ShooterSub;
import frc.robot.util.ShooterLookup;

public class RunShooterCMD extends Command {

    private final ShooterSub shooterSub;

    private double desiredVelocity; // fps
    private final boolean distanceBased;

    // 1‑param constructor
    // → distance-based: looks up velocity from Limelight distance when command starts
    public RunShooterCMD(ShooterSub shooterSub) {
        this.shooterSub = shooterSub;
        this.desiredVelocity = 0;
        this.distanceBased = true;

        addRequirements(shooterSub);
    }

    // 2‑param constructor
    // → desiredVelocity is passed explicitly
    public RunShooterCMD(ShooterSub shooterSub, double desiredVelocity) {
        this.shooterSub = shooterSub;
        this.desiredVelocity = desiredVelocity;
        this.distanceBased = false;

        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
        if (distanceBased) {
            double distanceFeet = getLimelightDistanceFeet();
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
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
