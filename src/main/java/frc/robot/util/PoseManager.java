package frc.robot.util;

import java.util.Optional;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.Pose2DConstants;
import frc.robot.subsystems.SwerveSub;

public class PoseManager {

    private double distMeters = 0;
    private Rotation2d heading;
    private double targetDeg = 0;
    private double errorDeg = 0;

    private Pose2d redHubPose2d = new Pose2d(Pose2DConstants.xHubPose, Pose2DConstants.yHubPose, 
        new Rotation2d());
        
    private Pose2d blueHubPose2d = new Pose2d(Pose2DConstants.xHubPose, Pose2DConstants.yHubPose, 
        new Rotation2d());

    SwerveSub swerveSub;

    public PoseManager(SwerveSub swerveSub){
        this.swerveSub = swerveSub;

    }

    public Pose2d getAllianceHubPose2d(){
       
        return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Blue ? blueHubPose2d : redHubPose2d)
        .orElse(blueHubPose2d); //If driver station is not detected by default it will be blue alliance
        
    }

    // returns inches away from target
    public double getDistanceToTargetInches(Pose2d targetPose){
        Pose2d robotPose = swerveSub.getPose();

        if(robotPose == null || targetPose == null) return -1.0;

        distMeters =  robotPose.getTranslation().getDistance(targetPose.getTranslation());

        return Units.metersToInches(distMeters);
    }

        /** Degrees to face target (0=along X-axis, CCW positive) */  
    public double getHeadingToTargetDegrees(Pose2d targetPose) {  
        Pose2d robotPose = swerveSub.getPose();  
        if (robotPose == null || targetPose == null) return 0.0;  
        
        Translation2d toTarget = targetPose.getTranslation().minus(robotPose.getTranslation());  
        heading = Rotation2d.fromRadians(Math.atan2(toTarget.getY(), toTarget.getX()));  
        return heading.getDegrees();  
    }  

    /** Error from current heading to target (degrees, -180 to 180) */  
    public double getHeadingErrorDegrees(Pose2d targetPose) {  
        Pose2d robotPose = swerveSub.getPose();  

        targetDeg = getHeadingToTargetDegrees(targetPose);  
        errorDeg = MathUtil.angleModulus(robotPose.getRotation().getDegrees() - targetDeg);

        return errorDeg;
    }   

    @Override
    public String toString(){
        String str = " ";

        str += "Pose Information: ";
        str += "\nInchesAwayFromTarget: " + Units.metersToInches(distMeters);
        str += "\nHeadingErrorDegrees: " +  errorDeg;
        str += "\nRotation2DRobotHeading: " + heading;

        return str;

    }

}
