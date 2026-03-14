package frc.robot.subsystems;

import java.util.Map;
import org.opencv.core.Mat;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodSub extends SubsystemBase{

    private final ServoHub servoHub;

    private final ServoChannel hoodLinActRightChannel;
    private final ServoChannel hoodLinActLeftChannel;

    public static double HoodAngle;
    public static double distanceFromLimelightToGoalInches;


    public HoodSub(){

        servoHub = new ServoHub(HoodConstants.kHoodId);

        ServoHubConfig config = new ServoHubConfig();

        config.channel0.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);

        config.channel1.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);
         

        servoHub.configure(config, ResetMode.kResetSafeParameters);

        hoodLinActRightChannel = servoHub.getServoChannel(ChannelId.kChannelId0);

        hoodLinActRightChannel.setEnabled(true);
        hoodLinActRightChannel.setPowered(true);

        hoodLinActLeftChannel = servoHub.getServoChannel(ChannelId.kChannelId1);

        hoodLinActLeftChannel.setEnabled(true);
        hoodLinActLeftChannel.setPowered(true);


    }

    public ServoChannel getHoodLinActRightChannel(){
        return hoodLinActRightChannel;
    }

    public ServoChannel getHoodLinActLeftChannel(){
        return hoodLinActLeftChannel;
    }

    public double getHoodAngle(){
        return HoodAngle;
    }

    public void setHoodAngle(double m_hoodAngle){
         HoodAngle = m_hoodAngle;
    }

    public double getLimelightToGoalInches(){
        return distanceFromLimelightToGoalInches;
    }

    public void setLimelightToGoalInches(double limelightToGoalInches){
        distanceFromLimelightToGoalInches = limelightToGoalInches;
    }

    private double map(
        double x,
        double inMin, double inMax,
        double outMin, double outMax
        ){

        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }

    private double inchesToPulseWidth(double extensionInches){
        MathUtil.clamp(extensionInches, Constants.HoodConstants.minExtensionInches, 
            Constants.HoodConstants.maxExtensionInches);

             return map(extensionInches, 
               HoodConstants.minExtensionInches, HoodConstants.maxExtensionInches,
               HoodConstants.kMinPulseUs,        HoodConstants.kMaxPulseUs);
    }

   private double degreeToInches(double angleDegree){
        return HoodConstants.a3 * Math.pow(angleDegree, 3) + HoodConstants.a2 * Math.pow(angleDegree, 2) + 
            HoodConstants.a1 * angleDegree + HoodConstants.a0;
   }


    

    @Override
    public String toString(){
        String str = " ";

        str += "DesiredHoodAngle" + getHoodAngle(); //to show desired hood angle
        str += "distanceAwayFromGoalInches" + getLimelightToGoalInches(); //how far we are away from goal

        return str;

    }
}
