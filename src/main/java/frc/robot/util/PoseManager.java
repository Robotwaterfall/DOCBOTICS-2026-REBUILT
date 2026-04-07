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
        new Pose2d(Pose2DConstants.xHubPose, Pose2DConstants.yHubPose, new Rotation2d());
    
    private static final Pose2d LEFT_ALLIANCE_ZONE_POSE =
        new Pose2d(Pose2DConstants.xAllianceZoneLeftPose, Pose2DConstants.yAllianceZoneLeftPose, 
            new Rotation2d());

    private static final Pose2d RIGHT_ALLIANCE_ZONE_POSE =
        new Pose2d(Pose2DConstants.xAllianceZoneRightPose, Pose2DConstants.yAllianceZoneRightPose, 
            new Rotation2d());

    /** Returns the hub pose for the current alliance */
    public static Pose2d getAllianceHubPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red
                ? FlippingUtil.flipFieldPose(HUB_POSE)
                : HUB_POSE)
            .orElse(HUB_POSE);
    }

    //Start of pose for shooting to alliance area
    public static Pose2d getLeftAllianceZonePose2d(){
        return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red
            ? FlippingUtil.flipFieldPose(LEFT_ALLIANCE_ZONE_POSE)
            : LEFT_ALLIANCE_ZONE_POSE)
            .orElse(LEFT_ALLIANCE_ZONE_POSE);
    }
    public static Pose2d getRightAllianceZonePose2d(){
        return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red
            ? FlippingUtil.flipFieldPose(RIGHT_ALLIANCE_ZONE_POSE)
            : RIGHT_ALLIANCE_ZONE_POSE)
            .orElse(RIGHT_ALLIANCE_ZONE_POSE);
    }
    //end of pose for shooting to alliance area

    /** True if the robot is inside the alliance zone */
    public static boolean isInAllianceZone(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        Translation2d blueMin = new Translation2d(
            Units.inchesToMeters(Pose2DConstants.ALLIANCE_ZONE_X_MIN_BLUEin),
            Units.inchesToMeters(Pose2DConstants.ALLIANCE_ZONE_Y_MIN_BLUEin)
        );

        Translation2d blueMax = new Translation2d(
            Units.inchesToMeters(Pose2DConstants.ALLIANCE_ZONE_X_MAX_BLUEin),
            Units.inchesToMeters(Pose2DConstants.ALLIANCE_ZONE_Y_MAX_BLUEin)
        );

        boolean isRed = DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red)
            .orElse(false);

        Translation2d min = isRed ? FlippingUtil.flipFieldPosition(blueMin) : blueMin;
        Translation2d max = isRed ? FlippingUtil.flipFieldPosition(blueMax) : blueMax;

        double minX = Math.min(min.getX(), max.getX());
        double maxX = Math.max(min.getX(), max.getX());
        double minY = Math.min(min.getY(), max.getY());
        double maxY = Math.max(min.getY(), max.getY());

        double x = robotPose.getX();
        double y = robotPose.getY();

        return x >= minX && x <= maxX && y >= minY && y <= maxY;
    }

    /** Returns inches away from a target pose */
    public static double getDistanceToTargetFeet(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        double distMeters =
            robotPose.getTranslation().getDistance(targetPose.getTranslation());

        return Units.metersToFeet(distMeters);
    }

    /** Returns inches away from the hub */
    public static double getDistanceToHubFeet(SwerveSub swerveSub) {
        return getDistanceToTargetFeet(swerveSub, getAllianceHubPose2d());
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

        // Convert to radians for angleModulus, then back to degrees
        double errorRad = MathUtil.angleModulus(
            Math.toRadians(targetDeg - robotDeg)  // ✅ target - robot, not robot - target
        );
        return Math.toDegrees(errorRad);
    }

    /** Returns heading error to the hub */
    public static double getHeadingErrorDegreesHub(SwerveSub swerveSub) {
        return getHeadingErrorDegrees(swerveSub, getAllianceHubPose2d());
    }
    
}