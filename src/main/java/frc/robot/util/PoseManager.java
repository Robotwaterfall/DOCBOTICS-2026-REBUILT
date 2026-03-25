package frc.robot.util;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Pose2DConstants;
import frc.robot.subsystems.SwerveSub;

public final class PoseManager {

    private PoseManager() {}

    private static final Pose2d HUB_POSE =
        new Pose2d(Pose2DConstants.xHubPose, Pose2DConstants.yHubPose, new Rotation2d());

    /** Returns the hub pose for the current alliance */
    public static Pose2d getAllianceHubPose2d() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red
                ? FlippingUtil.flipFieldPose(HUB_POSE)
                : HUB_POSE)
            .orElse(HUB_POSE);
    }

    /** True if the robot is inside the alliance zone */
    public static boolean isInAllianceZone(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null) return false;

        Pose2d fieldPose = FlippingUtil.flipFieldPose(robotPose);

        double x = fieldPose.getX();
        double y = fieldPose.getY();

        boolean inX = x >= Pose2DConstants.ALLIANCE_ZONE_X_MIN_BLUEin &&
                      x <= Pose2DConstants.ALLIANCE_ZONE_X_MAX_BLUEin;

        boolean inY = y >= Pose2DConstants.ALLIANCE_ZONE_Y_MIN_BLUEin &&
                      y <= Pose2DConstants.ALLIANCE_ZONE_Y_MAX_BLUEin;

        return inX && inY;
    }

    /** Returns inches away from a target pose */
    public static double getDistanceToTargetInches(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return -1.0;

        double distMeters =
            robotPose.getTranslation().getDistance(targetPose.getTranslation());

        return Units.metersToInches(distMeters);
    }

    /** Returns inches away from the hub */
    public static double getDistanceToHubInches(SwerveSub swerveSub) {
        return getDistanceToTargetInches(swerveSub, getAllianceHubPose2d());
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

        return MathUtil.angleModulus(robotDeg - targetDeg);
    }

    /** Returns heading error to the hub */
    public static double getHeadingErrorDegreesHub(SwerveSub swerveSub) {
        return getHeadingErrorDegrees(swerveSub, getAllianceHubPose2d());
    }
    
}