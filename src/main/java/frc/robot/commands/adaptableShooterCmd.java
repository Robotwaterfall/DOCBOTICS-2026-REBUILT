package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.ShooterSub;

public class adaptableShooterCmd extends Command{

    ShooterSub shooterSub;
    HoodSub hoodSub;

    private double desiredHoodAngle;

    private double distanceAwayFromGoal;

    private double desiredVelocity;

  
    

    public adaptableShooterCmd(ShooterSub shooterSub, HoodSub hoodSub) {

       this.shooterSub = shooterSub;
       this.hoodSub = hoodSub;

       desiredHoodAngle = hoodSub.getDesiredHoodAngle();
       distanceAwayFromGoal = hoodSub.getLimelightToGoalInches();

       desiredVelocity = shooterSub.getDesiredVelocity();

       addRequirements(shooterSub);
    }


    @Override
    public void initialize() {
        shooterSub.stopMotors();
        

    }

    @Override
    public void execute() {

        double theta = Math.toRadians(desiredHoodAngle);
        double distanceAwayMeters = Math.abs(distanceAwayFromGoal * Constants.unitConversions.inchesToMeters);


        double numerator = Constants.ShooterConstants.kGravity * Math.pow(distanceAwayMeters, 2);

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * 
                            (distanceAwayMeters * Math.tan(theta) - Constants.ShooterConstants.heightOfGoalMeters);

        if(denominator <= 0){
            SmartDashboard.putString("ShooterStatus: ", "CANT SHOOT. MOVE!!!");
            desiredVelocity = -1;
        }

        shooterSub.setDesiredVelocity(Math.sqrt(numerator/denominator));

        shooterSub.setShooterVelocityMPS(desiredVelocity);

        SmartDashboard.putNumber("DesiredVelocityOuput: ", desiredVelocity);




        if((shooterSub.getShooter_LeadVelocity()) > (desiredVelocity - ShooterConstants.shooterTolerance) &&
                (shooterSub.getShooter_LeadVelocity()) < (desiredVelocity + ShooterConstants.shooterTolerance)){

                    shooterSub.setIndexSpeed(ShooterConstants.indexSpeed);

                }

        
        
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    

}
