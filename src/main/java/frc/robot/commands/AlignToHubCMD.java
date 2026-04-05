package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class AlignToHubCMD extends Command {

    private final SwerveSub swerveSubsystem;

    public final Supplier<Double> xSpdFunction;
    public final Supplier<Double> ySpdFunction;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter turningLimiter;

    public static double CurrentXSpeed;
    public static double CurrentYSpeed;
    public static double CurrentTurningSpeed;
    public static boolean CurrentOrientation;

    public final double speed;

    public AlignToHubCMD(
            SwerveSub swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            double speed) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.speed = speed;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        double errorDeg = PoseManager.getHeadingErrorDegreesHub(swerveSubsystem);

        double kP = Constants.DriveConstants.autoTargetConstants.autoOrientKp;
        double turnCommand = MathUtil.clamp(errorDeg * kP, -Constants.DriveConstants.autoTargetConstants.autoOrientSpeed, 
            Constants.DriveConstants.autoTargetConstants.autoOrientSpeed
        );

        double turningSpeed = turningLimiter.calculate(turnCommand)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                -ySpeed,
                turningSpeed,
                swerveSubsystem.getRotation2d());

        CurrentXSpeed = xSpeed;
        CurrentYSpeed = ySpeed;
        CurrentTurningSpeed = turningSpeed;

        SwerveModuleState[] moduleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates, true);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}