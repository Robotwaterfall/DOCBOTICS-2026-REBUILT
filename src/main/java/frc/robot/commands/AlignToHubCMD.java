package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class AlignToHubCMD extends Command{

    PoseManager poseManager;
    SwerveSub swerveSub;

    public AlignToHubCMD(PoseManager poseManager, SwerveSub swerveSub){
        this.poseManager = poseManager;
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double headingErrorDeg = poseManager.getHeadingErrorDegreesHub();

        double rotSpeeds = Constants.SwerveModuleConstants.kTurning * headingErrorDeg;
        rotSpeeds = MathUtil.clamp(rotSpeeds, -Constants.SwerveModuleConstants.kMax_Rotational_Speed,
                                    Constants.SwerveModuleConstants.kMax_Rotational_Speed
        );
        
    }

    @Override
    public boolean isFinished(){
        return false;
        
    }

}
