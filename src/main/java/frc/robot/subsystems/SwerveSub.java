package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.util.PoseManager;
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

    public double distMeters;

    private Trajectory currentPPTrajectory;

    public SwerveSub() {
         // 1. Initialize Swerve Modules FIRST
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

    // 2. Initialize Gyro
    gyro = new Pigeon2(Constants.DriveConstants.kImuIdPort, "rio");

    // 3. Initialize Pose Estimator WITH VALID DATA (modules exist now)
    poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(0),  // Temp zero until gyro ready
        getModulePositionsAuto(),   // Safe now - modules initialized
        new Pose2d());

    // 4. PathPlanner Configuration (Safe)
    try {
        config = RobotConfig.fromGUISettings();
        if (config == null) {
            DriverStation.reportWarning("settings.json missing", false);
        }

                AutoBuilder.configure(
                this::getPose, // Robot pose supplier.
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose).
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE.
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds.
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your
                                                // Constants class.
                        new PIDConstants(PathPlannerConstants.kTranslationalKp, PathPlannerConstants.kTranslationalKi, 
                            PathPlannerConstants.kTranslationalKd), // Translation PID constants.

                        new PIDConstants(PathPlannerConstants.kRotationalKp, PathPlannerConstants.kRotationalKi, 
                            PathPlannerConstants.kRotationalKd) // Rotation PID constants.

                ),
                config,
                () -> {
                    /*
                     * Boolean supplier that controls when the path will be mirrored for the red
                     * alliance,
                     * This will flip the path being followed to the red side of the field.
                     * THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
                     */

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    } catch (Exception e) {
        DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), false);
    }

    // 5. Gyro Reset Thread
    new Thread(() -> {
        try {
            Thread.sleep(10900);
            zeroHeading();
        } catch (Exception e) {}
    }).start();

    // 6. Dashboard + Limelight
    SmartDashboard.putData("Field", m_Field);

    m_Field.getObject("PPTrajectory");

    resetTrajectory();
    
    // Reset pose after everything ready
    poseEstimator.resetPosition(getRotation2d(), getModulePositionsAuto(), new Pose2d());
    
    LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightFront, 0);
    LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightBackLeft, 0);
    }

    public void setCurrentPPTrajectory(Trajectory trajectory) {
        this.currentPPTrajectory = trajectory;
    }

    public Trajectory getCurrentPPTrajectory() {
        return currentPPTrajectory;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositionsAuto());

        fuseLimelight(LimelightConstants.LimelightFront);

        m_Field.setRobotPose(poseEstimator.getEstimatedPosition());

        if(currentPPTrajectory != null) {
            m_Field.getObject("PPTrajectory").setTrajectory(currentPPTrajectory);
        }


        SwerveModulePosition[] debugModulePosition = getModulePositionsAuto();
        for (int i = 0; i < debugModulePosition.length; i++) {
            SmartDashboard.putString("SwerveModulePostions [" + i + "]", 
                "distance: " + debugModulePosition[i].distanceMeters + " Angle: " + debugModulePosition[i].angle);
        }

        frontLeft.sendToDashboard();
        frontRight.sendToDashboard();
        backLeft.sendToDashboard();
        backRight.sendToDashboard();

        SmartDashboard.putNumber("RobotHeading: ", getHeading());
        SmartDashboard.putString("RobotLocation: ", getPose().getTranslation().toString());

        SmartDashboard.putNumber("ErrorToHub", PoseManager.getHeadingErrorDegreesHub(this));
        SmartDashboard.putBoolean("isInAllianceZone", PoseManager.isInAllianceZone(this));

        SmartDashboard.putNumber("Dist_To_HubFT", PoseManager.getDistanceToHubFeet(this));

        // // Limelight distance to target for shooter calibration
        // edu.wpi.first.math.geometry.Pose3d targetPose =
        //     LimelightHelpers.getTargetPose3d_CameraSpace(LimelightConstants.LimelightFront);
        // distMeters = Math.sqrt(
        //     targetPose.getX() * targetPose.getX() +
        //     targetPose.getY() * targetPose.getY() +
        //     targetPose.getZ() * targetPose.getZ());
        // SmartDashboard.putNumber("LL_Dist_ft", edu.wpi.first.math.util.Units.metersToInches(distMeters) / 12.0);
        // SmartDashboard.putNumber("LL_Dist_in", edu.wpi.first.math.util.Units.metersToInches(distMeters));
    }

    private void fuseLimelight(String limelightName) {
    if (!LimelightHelpers.getTV(limelightName)) return;

    // Feed gyro heading to Limelight for MegaTag2
    LimelightHelpers.SetRobotOrientation(limelightName,
        getHeading(), 0, 0, 0, 0, 0);

    // Always use wpiBlue — pose estimator works in blue-origin coordinates regardless of alliance
    LimelightHelpers.PoseEstimate poseEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (poseEst == null || poseEst.tagCount < 1) return;

    // Smart filtering
    if (poseEst.rawFiducials != null && poseEst.rawFiducials.length > 0 &&
        poseEst.rawFiducials[0].ambiguity < 0.7 &&
        poseEst.latency > 0.01) {

        double xyStdDev = limelightName.contains("Back") ? 1.0 : 0.5;
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 1000));

        poseEstimator.addVisionMeasurement(poseEst.pose, poseEst.timestampSeconds);

        SmartDashboard.putNumber(limelightName + "_tagCount", poseEst.tagCount);
        SmartDashboard.putNumber(limelightName + "_ambiguity", poseEst.rawFiducials[0].ambiguity);
    }
}


    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public double getDistMeters(){
        return distMeters;
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositionsAuto(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void resetTrajectory(){
        currentPPTrajectory = null;
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
        // On red alliance, "forward" is 180° in the blue-origin coordinate system
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            gyro.setYaw(180);
        }
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