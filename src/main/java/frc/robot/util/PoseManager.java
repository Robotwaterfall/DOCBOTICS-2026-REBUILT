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

    private static final Pose2d BLUE_HUB_POSE =
        new Pose2d(
            Pose2DConstants.BLUE_HUB_POSE_X_M,
            Pose2DConstants.BLUE_HUB_POSE_Y_M,
            new Rotation2d()
        );

    /** Driver Prespective */
    private static final Pose2d BLUE_ZONE_LEFT_MIDPOINT_POSE =
        new Pose2d(
            Pose2DConstants.BLUE_ZONE_MIDPOINT_X_M,
            Pose2DConstants.BLUE_ZONE_LEFT_MIDPOINT_Y_M,
            new Rotation2d()
        );

    /** Driver Prespective */
    private static final Pose2d BLUE_ZONE_RIGHT_MIDPOINT_POSE =
        new Pose2d(
            Pose2DConstants.BLUE_ZONE_MIDPOINT_X_M,
            Pose2DConstants.BLUE_ZONE_RIGHT_MIDPOINT_Y_M,
            new Rotation2d()
        );

    /** Returns the hub pose for the current alliance */
    public static Pose2d getAllianceHubPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(BLUE_HUB_POSE)
                : BLUE_HUB_POSE)
            .orElse(BLUE_HUB_POSE);
    }

    /** Returns the left alliance zone reference pose for the current alliance */
    /** Driver Prespective */
    public static Pose2d getLeftAllianceZoneMidpointPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(BLUE_ZONE_LEFT_MIDPOINT_POSE)
                : BLUE_ZONE_LEFT_MIDPOINT_POSE)
            .orElse(BLUE_ZONE_LEFT_MIDPOINT_POSE);
    }

    /** Returns the right alliance zone reference pose for the current alliance */
    /** Driver Prespective */
    public static Pose2d getRightAllianceZoneMidpointPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red
                ? FlippingUtil.flipFieldPose(BLUE_ZONE_RIGHT_MIDPOINT_POSE)
                : BLUE_ZONE_RIGHT_MIDPOINT_POSE)
            .orElse(BLUE_ZONE_RIGHT_MIDPOINT_POSE);
    }

    /** True if the robot is inside its alliance zone */
    public static boolean isInAllianceZone(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        Translation2d blueMin = new Translation2d(
            Pose2DConstants.BLUE_ZONE_X_MIN_M,
            Pose2DConstants.BLUE_ZONE_Y_MIN_M
        );

        Translation2d blueMax = new Translation2d(
            Pose2DConstants.BLUE_ZONE_X_MAX_M,
            Pose2DConstants.BLUE_ZONE_Y_MAX_M
        );

        boolean isRed = DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red)
            .orElse(false);

        Translation2d allianceMin = isRed ? FlippingUtil.flipFieldPosition(blueMin) : blueMin;
        Translation2d allianceMax = isRed ? FlippingUtil.flipFieldPosition(blueMax) : blueMax;

        double minX = Math.min(allianceMin.getX(), allianceMax.getX());
        double maxX = Math.max(allianceMin.getX(), allianceMax.getX());
        double minY = Math.min(allianceMin.getY(), allianceMax.getY());
        double maxY = Math.max(allianceMin.getY(), allianceMax.getY());

        double x = robotPose.getX();
        double y = robotPose.getY();

        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
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

        return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
    }

    /** True if the robot is on the left side of the field, split by the center line (alliance-aware) */
    public static boolean isOnLeftSideOfField(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        double y = robotPose.getY();

        return y >= Pose2DConstants.HALF_FIELD_Y_M;
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
        return getDistanceToTargetFeet(swerveSub, getLeftAllianceZoneMidpointPose2d());
    }

     /** Returns feet away from the right alliance zone area where fuel can collect */
    /**  Outtpost side */
    public static double getDistanceToRightAllianceZone(SwerveSub swerveSub){
        return getDistanceToTargetFeet(swerveSub, getRightAllianceZoneMidpointPose2d());
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
        return getHeadingErrorDegrees(swerveSub, getLeftAllianceZoneMidpointPose2d());
    }

    public static double getHeadingErrorDegreesRightAllianceZoneArea(SwerveSub swerveSub){
        return getHeadingErrorDegrees(swerveSub, getRightAllianceZoneMidpointPose2d());
    }
}