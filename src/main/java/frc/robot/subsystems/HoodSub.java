package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.hoodConstants;

public class HoodSub extends SubsystemBase{

    private final ServoHub servoHub;

    private final ServoChannel hoodServo;


    public HoodSub(){

        servoHub = new ServoHub(hoodConstants.kHoodId);

        ServoHubConfig config = new ServoHubConfig();
         config.channel0.pulseRange(Constants.hoodConstants.kMinPulseUs, Constants.hoodConstants.kCenterPulseUs, 
         Constants.hoodConstants.kMaxPulseUs);

         servoHub.configure(config, ResetMode.kResetSafeParameters);

        hoodServo = servoHub.getServoChannel(ChannelId.kChannelId0);

        hoodServo.setEnabled(true);
        hoodServo.setPowered(true);

    }

    public ServoChannel getHoodServo(){
        return hoodServo;
    }

     /** Set hood to a target angle in degrees. */
    public void setHoodAngle(double degrees) {
        // Clamp to allowed range
        double clampedDeg = Math.max(Constants.hoodConstants.kMinAngleDeg, Math.min(Constants.hoodConstants.kMaxAngleDeg, degrees));

        // Map [minDeg, maxDeg] → [minPulse, maxPulse]
        double t = (clampedDeg - Constants.hoodConstants.kMinAngleDeg) / (
            Constants.hoodConstants.kMaxAngleDeg - Constants.hoodConstants.kMinAngleDeg);
        int pulseUs = (int) (Constants.hoodConstants.kMinPulseUs + t * (Constants.hoodConstants.kMaxPulseUs - 
        Constants.hoodConstants.kMinPulseUs));

        hoodServo.setPulseWidth(pulseUs);
    }
}
