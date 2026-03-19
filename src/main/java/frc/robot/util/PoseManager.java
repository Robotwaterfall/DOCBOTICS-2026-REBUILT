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

    // Prevent instantiation
    private PoseManager() {}

    // Static state (optional but preserved from original class)
    private static double distMeters = 0;
    private static Rotation2d heading = new Rotation2d();
    private static double targetDeg = 0;
    private static double errorDeg = 0;

    private static final Pose2d HubPose2d = new Pose2d(Pose2DConstants.xHubPose,
        Pose2DConstants.yHubPose,new Rotation2d()
    );

    /** Returns the hub pose for the current alliance */
    public static Pose2d getAllianceHubPose2d() {
    
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red 
            ? FlippingUtil.flipFieldPose(HubPose2d) 
            : HubPose2d)
        .orElse(HubPose2d);
}

    /** Returns inches away from target */
    public static double getDistanceToTargetInches(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return -1.0;

        distMeters = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        return Units.metersToInches(distMeters);
    }

    /** Returns inches away from hub */
    public static double getDistanceToHubInches(SwerveSub swerveSub) {
        Pose2d targetPose = getAllianceHubPose2d();
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return -1.0;

        distMeters = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        return Units.metersToInches(distMeters);
    }

    /** Degrees to face target (0 = along X-axis, CCW positive) */
    public static double getHeadingToTargetDegrees(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        Translation2d toTarget =
            targetPose.getTranslation().minus(robotPose.getTranslation());

        heading = Rotation2d.fromRadians(Math.atan2(toTarget.getY(), toTarget.getX()));
        return heading.getDegrees();
    }

    /** Error from current heading to target (degrees, -180 to 180) */
    public static double getHeadingErrorDegrees(SwerveSub swerveSub, Pose2d targetPose) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || targetPose == null) return 0.0;

        targetDeg = getHeadingToTargetDegrees(swerveSub, targetPose);
        errorDeg = MathUtil.angleModulus(robotPose.getRotation().getDegrees() - targetDeg);

        return errorDeg;
    }

     public static double getHeadingErrorDegreesHub(SwerveSub swerveSub) {
        Pose2d robotPose = swerveSub.getPose();
        if (robotPose == null || getAllianceHubPose2d() == null) return 0.0;

        targetDeg = getHeadingToTargetDegrees(swerveSub, getAllianceHubPose2d());
        errorDeg = MathUtil.angleModulus(robotPose.getRotation().getDegrees() - targetDeg);

        return errorDeg;
    }

    /** Debug string */
    @Override
    public String toString() {
        return "Pose Information:"
            + "\nInchesAwayFromTarget: " + Units.metersToInches(distMeters)
            + "\nHeadingErrorDegrees: " + errorDeg
            + "\nRotation2DRobotHeading: " + heading;
    }
}