package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class AlignToHubCMD extends Command{

    SwerveSub swerveSub;

    public AlignToHubCMD(SwerveSub swerveSub){
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double headingErrorDeg = PoseManager.getHeadingErrorDegreesHub(swerveSub);

        double rotSpeeds = Constants.LockOnPoseConstants.kTurning * Math.toRadians(headingErrorDeg);
        rotSpeeds = MathUtil.clamp(rotSpeeds, -Constants.LockOnPoseConstants.kMax_Rotational_Speed,
                                    Constants.LockOnPoseConstants.kMax_Rotational_Speed
        );

        swerveSub.driveRobotRelative(new ChassisSpeeds(0, 0, rotSpeeds));;
        
    }

    @Override
    public boolean isFinished(){
        return Math.abs(PoseManager.getHeadingErrorDegreesHub(swerveSub)) 
                            <= Constants.LockOnPoseConstants.headingToleranceDeg;
        
    }

    @Override
    public void end(boolean interrupted){
        swerveSub.driveRobotRelative(new ChassisSpeeds(0,0,0));
    }

}
