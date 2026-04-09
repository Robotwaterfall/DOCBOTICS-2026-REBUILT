package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Pose2DConstants;
import frc.robot.subsystems.SwerveSub;

public final class PoseManager {

    private PoseManager() {}

    private static final Pose2d HUB_POSE =
        new Pose2d(
            Pose2DConstants.X_HUB_POSE_M,
            Pose2DConstants.Y_HUB_POSE_M,
            new Rotation2d()
        );

    /** Driver Prespective */
    private static final Pose2d LEFT_ALLIANCE_ZONE_POSE =
        new Pose2d(
            Pose2DConstants.X_ALLIANCE_ZONE_LEFT_M,
            Pose2DConstants.Y_ALLIANCE_ZONE_LEFT_M,
            new Rotation2d()
        );

    /** Driver Prespective */
    private static final Pose2d RIGHT_ALLIANCE_ZONE_POSE =
        new Pose2d(
            Pose2DConstants.X_ALLIANCE_ZONE_RIGHT_M,
            Pose2DConstants.Y_ALLIANCE_ZONE_RIGHT_M,
            new Rotation2d()
        );

    /** Returns the hub pose for the current alliance */
    public static Pose2d getAllianceHubPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(HUB_POSE)
                : HUB_POSE)
            .orElse(HUB_POSE);
    }

    /** Returns the left alliance zone reference pose for the current alliance */
    /** Driver Prespective */
    public static Pose2d getLeftAllianceZonePose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(LEFT_ALLIANCE_ZONE_POSE)
                : LEFT_ALLIANCE_ZONE_POSE)
            .orElse(LEFT_ALLIANCE_ZONE_POSE);
    }

    /** Returns the right alliance zone reference pose for the current alliance */
    /** Driver Prespective */
    public static Pose2d getRightAllianceZonePose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(RIGHT_ALLIANCE_ZONE_POSE)
                : RIGHT_ALLIANCE_ZONE_POSE)
            .orElse(RIGHT_ALLIANCE_ZONE_POSE);
    }

    /** True if the robot is inside its alliance zone */
    public static boolean isInAllianceZone(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        Translation2d blueMin = new Translation2d(
            Pose2DConstants.ALLIANCE_ZONE_X_MIN_BLUE_M,
            Pose2DConstants.ALLIANCE_ZONE_Y_MIN_BLUE_M
        );

        Translation2d blueMax = new Translation2d(
            Pose2DConstants.ALLIANCE_ZONE_X_MAX_BLUE_M,
            Pose2DConstants.ALLIANCE_ZONE_Y_MAX_BLUE_M
        );

        boolean isRed = DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red)
            .orElse(false);

        Translation2d flippedA = isRed ? FlippingUtil.flipFieldPosition(blueMin) : blueMin;
        Translation2d flippedB = isRed ? FlippingUtil.flipFieldPosition(blueMax) : blueMax;

        double minX = Math.min(flippedA.getX(), flippedB.getX());
        double maxX = Math.max(flippedA.getX(), flippedB.getX());
        double minY = Math.min(flippedA.getY(), flippedB.getY());
        double maxY = Math.max(flippedA.getY(), flippedB.getY());

        double x = robotPose.getX();
        double y = robotPose.getY();

        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** True if the robot is inside the neutral zone / wasteland */
    public static boolean isInWasteLand(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        double minX = Pose2DConstants.NEUTRAL_ZONE_X_MIN_M;
        double maxX = Pose2DConstants.NEUTRAL_ZONE_X_MAX_M;
        double minY = Pose2DConstants.NEUTRAL_ZONE_Y_MIN_M;
        double maxY = Pose2DConstants.NEUTRAL_ZONE_Y_MAX_M;

        double x = robotPose.getX();
        double y = robotPose.getY();

        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

/** True if the robot is on the left side of the field, split by the center line (alliance-aware) */
    public static boolean isOnLeftSideOfField(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        double y = robotPose.getY();

        return y >= Pose2DConstants.HALF_FIELD_Y_MAX_M;
    }

    /** Returns feet away from a target pose */
    public static double getDistanceToTargetFeet(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        double distMeters =
            robotPose.getTranslation().getDistance(targetPose.getTranslation());

        return Units.metersToFeet(distMeters);
    }

    /** Returns feet away from the hub */
    public static double getDistanceToHubFeet(SwerveSub swerveSub) {
        return getDistanceToTargetFeet(swerveSub, getAllianceHubPose2d());
    }

    /** Returns feet away from the left alliance zone area where fuel can collect */
    /** Depot side */
    public static double getDistanceToLeftAllianceZone(SwerveSub swerveSub){
        return getDistanceToTargetFeet(swerveSub, getLeftAllianceZonePose2d());
    }

     /** Returns feet away from the right alliance zone area where fuel can collect */
    /**  Outtpost side */
    public static double getDistanceToRightAllianceZone(SwerveSub swerveSub){
        return getDistanceToTargetFeet(swerveSub, getRightAllianceZonePose2d());
    }

    /** Returns the heading (degrees) from robot to target */
    public static double getHeadingToTargetDegrees(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        Translation2d toTarget =
            targetPose.getTranslation().minus(robotPose.getTranslation());

        return Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
    }

    /** Returns heading error (degrees, -180 to 180) from robot to target */
    public static double getHeadingErrorDegrees(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        double targetDeg = getHeadingToTargetDegrees(swerveSub, targetPose);
        double robotDeg = robotPose.getRotation().getDegrees();

        double errorRad = MathUtil.angleModulus(
            Math.toRadians(targetDeg - robotDeg)
        );

        return Math.toDegrees(errorRad);
    }

    /** Returns heading error to the hub */
    public static double getHeadingErrorDegreesHub(SwerveSub swerveSub) {
        return getHeadingErrorDegrees(swerveSub, getAllianceHubPose2d());
    }
    
    public static double getHeadingErrorDegreesLeftAllianceZoneArea(SwerveSub swerveSub){
        return getHeadingErrorDegrees(swerveSub, getLeftAllianceZonePose2d());
    }

    public static double getHeadingErrorDegreesRightAllianceZoneArea(SwerveSub swerveSub){
        return getHeadingErrorDegrees(swerveSub, getRightAllianceZonePose2d());
    }
}