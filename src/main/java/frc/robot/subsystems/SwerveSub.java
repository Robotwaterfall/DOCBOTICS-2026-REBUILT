package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.config.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveSub extends SubsystemBase {
    // Hardware Modules
    public final SwerveModule frontRight;
    public final SwerveModule frontLeft;
    public final SwerveModule backRight;
    public final SwerveModule backLeft;
    private final SwerveModule[] swerveModules;

    // Sensors and Logic
    private final Pigeon2 gyro;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d m_Field = new Field2d();
    private RobotConfig config;

    public SwerveSub() {
        // 1. Initialize Swerve Modules
        frontRight = new SwerveModule(
                DriveConstants.kFrontRightDriveMotorPort,
                DriveConstants.kFrontRightTurningMotorPort,
                DriveConstants.kFrontRightDriveEncoderReversed,
                DriveConstants.kFrontRightTurningEncoderReversed,
                DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        frontLeft = new SwerveModule(
                DriveConstants.kFrontLeftDriveMotorPort,
                DriveConstants.kFrontLeftTurningMotorPort,
                DriveConstants.kFrontLeftDriveEncoderReversed,
                DriveConstants.kFrontLeftTurningEncoderReversed,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        backRight = new SwerveModule(
                DriveConstants.kBackRightDriveMotorPort,
                DriveConstants.kBackRightTurningMotorPort,
                DriveConstants.kBackRightDriveEncoderReversed,
                DriveConstants.kBackRightTurningEncoderReversed,
                DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

        backLeft = new SwerveModule(
                DriveConstants.kBackLeftDriveMotorPort,
                DriveConstants.kBackLeftTurningMotorPort,
                DriveConstants.kBackLeftDriveEncoderReversed,
                DriveConstants.kBackLeftTurningEncoderReversed,
                DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
                DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        swerveModules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

        // 2. Initialize Gyro (Must be before PoseEstimator)
        gyro = new Pigeon2(Constants.DriveConstants.kImuIdPort, "rio");

        // 3. Initialize Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                getRotation2d(),
                getModulePositionsAuto(),
                new Pose2d());

        // 4. PathPlanner Configuration
        try {
            config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose, 
                this::resetPose, 
                this::getSpeeds, 
                (speeds, feedforwards) -> driveRobotRelative(speeds), 
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0) 
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this 
            );
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config could not be loaded! Check your deploy folder: " + e.getMessage(), e.getStackTrace());
        }

        // Gyro Reset Thread
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {}
        }).start();

        SmartDashboard.putData("Field", m_Field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositionsAuto());

        // Vision Updates
        boolean doRejectUpdate = LimelightHelpers.getTV(LimelightConstants.Limelight3);
        if (doRejectUpdate) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.Limelight3);
            if (mt1.tagCount > 0) {
                poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }

        // Logging and Telemetry
        Logger.recordOutput("RobotPose", poseEstimator.getEstimatedPosition());
        Logger.recordOutput("heading", getHeading());
        
        SmartDashboard.putNumber("robot Heading", getHeading());
        SmartDashboard.putString("robot location", getPose().getTranslation().toString());

        SwerveModulePosition[] debugModulePosition = getModulePositionsAuto();
        for (int i = 0; i < debugModulePosition.length; i++) {
            SmartDashboard.putString("SwerveModulePostions [" + i + "]", 
                "distance: " + debugModulePosition[i].distanceMeters + " Angle: " + debugModulePosition[i].angle);
        }

        frontLeft.sendToDashboard();
        frontRight.sendToDashboard();
        backLeft.sendToDashboard();
        backRight.sendToDashboard();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositionsAuto(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates, true);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isDeadband) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontRight.setDesiredState(desiredStates[0], isDeadband);
        frontLeft.setDesiredState(desiredStates[1], isDeadband);
        backRight.setDesiredState(desiredStates[2], isDeadband);
        backLeft.setDesiredState(desiredStates[3], isDeadband);

        // Logging Current vs Desired
        SwerveModuleState[] currentStates = getModuleStates();
        Logger.recordOutput("CurrentStates", currentStates);
        Logger.recordOutput("DesiredStates", desiredStates);
    }

    public SwerveModulePosition[] getModulePositionsAuto() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getSwerveModulePosition();
        }
        return positions;
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        double angleDeg = gyro.getRotation2d().getDegrees();
        return Math.IEEEremainder(-angleDeg, 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }
}