package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodSub extends SubsystemBase{

    private final ServoHub servoHub;

    private final ServoChannel hoodServoPrimaryChannel;
    private final ServoChannel hoodServoSecondaryChannel;

    public static double desiredHoodAngle;
    public static double distanceFromLimelightToGoalInches;


    public HoodSub(){

        servoHub = new ServoHub(HoodConstants.kHoodId);

        ServoHubConfig config = new ServoHubConfig();

        config.channel0.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);

        config.channel1.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);
         

        servoHub.configure(config, ResetMode.kResetSafeParameters);

        hoodServoPrimaryChannel = servoHub.getServoChannel(ChannelId.kChannelId0);

        hoodServoPrimaryChannel.setEnabled(true);
        hoodServoPrimaryChannel.setPowered(true);

        hoodServoSecondaryChannel = servoHub.getServoChannel(ChannelId.kChannelId1);

        hoodServoSecondaryChannel.setEnabled(true);
        hoodServoSecondaryChannel.setPowered(true);


    }

    public ServoChannel getHoodServoPrimaryChannel(){
        return hoodServoPrimaryChannel;
    }

    public ServoChannel getHoodServoSecondaryChannel(){
        return hoodServoSecondaryChannel;
    }

    public double getDesiredHoodAngle(){
        return desiredHoodAngle;
    }

    public void setDesiredHoodAngle(double desiredHoodAngleDegrees){
         desiredHoodAngle = desiredHoodAngleDegrees;
    }

    public double getLimelightToGoalInches(){
        return distanceFromLimelightToGoalInches;
    }

    public void setLimelightToGoalInches(double limelightToGoalInches){
        distanceFromLimelightToGoalInches = limelightToGoalInches;
    }

     /** Set hood to a target angle in degrees. */
    public void setHoodAngle(double degrees) {
        // Clamp to allowed range
        double clampedDeg = Math.max(Constants.HoodConstants.kMinAngleDeg, Math.min(Constants.HoodConstants.kMaxAngleDeg, degrees));

        // Map [minDeg, maxDeg] → [minPulse, maxPulse]
        double t = (clampedDeg - Constants.HoodConstants.kMinAngleDeg) / (
            Constants.HoodConstants.kMaxAngleDeg - Constants.HoodConstants.kMinAngleDeg);
        int pulseUs = (int) (Constants.HoodConstants.kMinPulseUs + t * (Constants.HoodConstants.kMaxPulseUs - 
        Constants.HoodConstants.kMinPulseUs));

        hoodServoPrimaryChannel.setPulseWidth(pulseUs);
        hoodServoSecondaryChannel.setPulseWidth(pulseUs);

    }
}
