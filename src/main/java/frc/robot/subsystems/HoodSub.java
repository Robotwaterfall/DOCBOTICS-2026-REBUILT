package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodSub extends SubsystemBase{

    private final ServoHub servoHub;

    private final ServoChannel hoodLinActRightChannel;
    private final ServoChannel hoodLinActLeftChannel;

    private double hoodAngle;
    private double distanceFromLimelightToGoalInches;

    private double tunableHoodAngle = Constants.HoodConstants.kHoodWarmUpDeg; // Start at a safe angle to avoid collisions

    public HoodSub(){
        servoHub = new ServoHub(HoodConstants.kHoodId);

        ServoHubConfig config = new ServoHubConfig();

        config.channel0.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);

        config.channel1.pulseRange(Constants.HoodConstants.kMinPulseUs, Constants.HoodConstants.kCenterPulseUs, 
        Constants.HoodConstants.kMaxPulseUs);
         

        servoHub.configure(config, ResetMode.kResetSafeParameters);

        hoodLinActRightChannel = servoHub.getServoChannel(ChannelId.kChannelId0);

        // Channels MUST be enabled and powered to use .setPulseWidth
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
        return hoodAngle;
    }

    public double getLimelightToGoalInches(){
        return distanceFromLimelightToGoalInches;
    }

    public void setLimelightToGoalInches(double limelightToGoalInches){
        distanceFromLimelightToGoalInches = limelightToGoalInches;
    }

    // Primary service method for HoodServoAdjustCMD
    public void setHoodAngle(double hoodAngle){
        this.hoodAngle = hoodAngle;
        int desiredPulseWidth = inchesToPulseWidth(degreeToInches(hoodAngle));
        
        hoodLinActLeftChannel.setPulseWidth(desiredPulseWidth);
        hoodLinActRightChannel.setPulseWidth(desiredPulseWidth);
    }

    // Scalar mapping for inches to pulse width to use
    private double map(
        double x,
        double inMin, double inMax,
        double outMin, double outMax
        ){
        return outMin + (x - inMin) * (outMax - outMin) / (inMax - inMin);
    }
    
    // int return type required because .setPulseWidth has int param

    private int inchesToPulseWidth(double extensionInches){
        extensionInches = MathUtil.clamp(extensionInches, Constants.HoodConstants.minExtensionInches,
            Constants.HoodConstants.maxExtensionInches);
        
        return (int)map(extensionInches, // Casting done *here* instead of map method to avoid int division -> avoids excessive rounding
            HoodConstants.minExtensionInches, HoodConstants.maxExtensionInches,
            HoodConstants.kMinPulseUs,        HoodConstants.kMaxPulseUs);
    }

   private double degreeToInches(double angleDegree){
        // This is the cubic approx for the path the hood takes
        return HoodConstants.a3 * Math.pow(angleDegree, 3) + HoodConstants.a2 * Math.pow(angleDegree, 2) + 
            HoodConstants.a1 * angleDegree + HoodConstants.a0;
   }

   public boolean isAtSetAngle(){
        int desiredPulseWidth = inchesToPulseWidth(degreeToInches(hoodAngle));
        int currentLeftPulseWidth = hoodLinActLeftChannel.getPulseWidth();
        int currentRightPulseWidth = hoodLinActRightChannel.getPulseWidth();

        return Math.abs(currentLeftPulseWidth - desiredPulseWidth) <= Constants.HoodConstants.kHoodToleranceUs &&
            Math.abs(currentRightPulseWidth - desiredPulseWidth) <= Constants.HoodConstants.kHoodToleranceUs;
   }

   public void incrementAngle(double deltaDegrees) {
        tunableHoodAngle += deltaDegrees;
        setHoodAngle(tunableHoodAngle);  // Applies via your pulse logic
    }

    public void decrementAngle(double deltaDegrees) {
        tunableHoodAngle -= deltaDegrees;
        setHoodAngle(tunableHoodAngle);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("DesiredHoodAngle", getHoodAngle());
        SmartDashboard.putNumber("TunableHoodAngle", tunableHoodAngle);
        SmartDashboard.putNumber("CurrentLeftPulseWidth", hoodLinActLeftChannel.getPulseWidth());
        SmartDashboard.putNumber("CurrentRightPulseWidth", hoodLinActRightChannel.getPulseWidth());
        SmartDashboard.putBoolean("isAtSetAngle", isAtSetAngle());
    }
}