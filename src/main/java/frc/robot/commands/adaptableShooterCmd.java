package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.ShooterSub;

/*
 * Do the calculations in ShooterPhysics.java for readability. Make sure all units are in inches.
 * 
 * This shoot command should grab ideal velocity from ShooterPhysics and set the motors to this velocity.
 * 
 * Flutter intake to help index into shooter maybe...
 * 
 */

public class AdaptableShooterCMD extends Command{

    ShooterSub shooterSub;
    HoodSub hoodSub;

    private double desiredHoodAngle;

    private double distanceAwayFromGoal;

    private double desiredVelocity;

    private Supplier<Double> shootSupplier;

    private boolean isAuto;

  
    

    public AdaptableShooterCMD(ShooterSub shooterSub, HoodSub hoodSub, 
                                Supplier<Double> shootSupplier, boolean isAuto) {

       this.shooterSub = shooterSub;
       this.hoodSub = hoodSub;

       this.shootSupplier = shootSupplier;
       this.isAuto = isAuto;

       desiredHoodAngle = hoodSub.getHoodAngle();
       distanceAwayFromGoal = hoodSub.getLimelightToGoalInches();

       desiredVelocity = shooterSub.getDesiredVelocity();

       addRequirements(shooterSub);
    }


    @Override
    public void initialize() {

        //before command starts stop the motors
        shooterSub.stopShooterMotors();
        

    }

    @Override
    public void execute() {

        double theta = Math.toRadians(desiredHoodAngle);
        double distanceAwayMeters = Math.abs(distanceAwayFromGoal * Constants.unitConversions.inchesToMeters);

        double numerator = Constants.ShooterConstants.kGravity * Math.pow(distanceAwayMeters, 2);

        double denominator = 2 * Math.pow(Math.cos(theta), 2) * 
                            (distanceAwayMeters * Math.tan(theta) - Constants.ShooterConstants.heightOfGoalMeters);

        if(denominator <= 0){
            SmartDashboard.putString("ShooterStatus: ", "IMPOSIBLE SHOT. MOVE!!!");
            desiredVelocity = -1;
        }

        shooterSub.setDesiredVelocity(Math.sqrt(numerator/denominator));

        shooterSub.setShooterVelocityMPS(desiredVelocity);

       



        //If the velocity is within tolerance start to transfer the balls from holding towards shooter
        if((shooterSub.getShooterLeadVelocity()) > (desiredVelocity - ShooterConstants.shooterTolerance) &&
                (shooterSub.getShooterLeadVelocity()) < (desiredVelocity + ShooterConstants.shooterTolerance)){

                    shooterSub.setIndexSpeed(ShooterConstants.indexSpeed);

                }

        
        
    }

    @Override
    public void end(boolean interrupted){
        //when the command ends stop motors and set velocity to 0
        shooterSub.setDesiredVelocity(0);
        shooterSub.stopShooterMotors(); 

    }


    @Override
    public boolean isFinished() {
        return shootSupplier.get() <= 0.3 && !isAuto; //if the robot is in auto we dont want the shooter to stop
    }

    

}
