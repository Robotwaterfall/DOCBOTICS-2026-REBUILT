package frc.robot.commands;

import com.revrobotics.servohub.ServoChannel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSub;

public class hoodServoAdjustCmd extends Command{

    private final HoodSub hoodSub;
    ServoChannel hoodServo;

    private double distanceFromLimelightToGoalInches;
    public static double desiredHoodAngle;


    public hoodServoAdjustCmd(HoodSub hoodSub){
        this.hoodSub = hoodSub;
        this.hoodServo = hoodSub.getHoodServo(); 
        addRequirements(hoodSub);

    }

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){

            // limelight distance calculations 
            NetworkTable table = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.Limelight2);
            NetworkTableEntry ty = table.getEntry("ty");
            double targetOffsetAngle_Vertical = ty.getDouble(0.0);

            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = Constants.LimelightConstants.limelightMountAngleDegrees; 

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = Constants.LimelightConstants.limelightLensHeightInches; 

            // distance from the target to the floor
            double goalHeightInches = Constants.LimelightConstants.goalHeightInches; 

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);


            //calculate distance
            distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            SmartDashboard.putNumber("distanceAwayFromGoal", distanceFromLimelightToGoalInches);



            //Find Angle hood needs to be at
            desiredHoodAngle = Math.atan(Constants.LimelightConstants.goalHeightInches/distanceFromLimelightToGoalInches);
            SmartDashboard.putNumber("desiredHoodAngle", desiredHoodAngle);

            //Set the angle of the hood
            hoodSub.setHoodAngle(desiredHoodAngle);

    }

   

}
