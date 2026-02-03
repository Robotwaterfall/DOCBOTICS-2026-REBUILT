package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.SwerveSub;

public class SwerveLimelightLockCmd extends Command {
   
      private final SwerveSub swerveSubsystem;

      public final Supplier<Double> xSpdFunction;
      public final Supplier<Double> ySpdFunction;
      private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; // slew rate limiter cap the the amount of change of a value

      public static double CurrentXSpeed;
      public static double CurrentYSpeed;
      public static double CurrentTurningSpeed;
      public static boolean CurrentOrientation;
      public final double speed;
    //   private final SwerveModuleState[] desiredLockOnStates = new SwerveModuleState[]{

    //     new SwerveModuleState(0, new Rotation2d(-0.394* 2 * Math.PI)), // front right
    //     new SwerveModuleState(0, new Rotation2d(-0.489 * 2 * Math.PI)), // front left
    //     new SwerveModuleState(0, new Rotation2d(0.25 * 2 * Math.PI)),// back right
    //     new SwerveModuleState(0, new Rotation2d(-0.1246 * 2 * Math.PI)) // back left

    // };


    public SwerveLimelightLockCmd( SwerveSub swerveSubsystem, 
          Supplier <Double> xSpdFunction,
           Supplier<Double> ySpdFunction, 
           double speed) { 
        
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;

        this.speed = speed;

 
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

  }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

      double lockonAngularVelocity = LimelightHelpers.getTX(LimelightConstants.Limelight2) * DriveConstants.autoTargetConstants.autoLockKp;

      lockonAngularVelocity *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

      lockonAngularVelocity *= -1.0;

      // gett latest values from joystick
      //swerveSubsystem.orientToTarget();
      double xspeed = xSpdFunction.get();
      double yspeed = ySpdFunction.get();

      double turningSpeed = lockonAngularVelocity; //TODO: output PID to turningspeed from TX of limelight
      
      //now apply deadband,  if joystick does not center back to exactly zero, it still stops
      xspeed = Math.abs(xspeed) > OIConstants.kDeadband ? xspeed : 0.0;
      yspeed = Math.abs(yspeed) > OIConstants.kDeadband ? yspeed : 0.0;

      // allows for violent joystick movements to be more smooth

      xspeed = xLimiter.calculate(xspeed) *  DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
      yspeed = yLimiter.calculate(yspeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
      turningSpeed = turningLimiter.calculate(turningSpeed) *
      DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

      
      ChassisSpeeds chassisSpeeds;
  
          chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xspeed, -yspeed, -turningSpeed, swerveSubsystem.getRotation2d());

      
      CurrentXSpeed = xspeed;
      CurrentYSpeed = yspeed;
      CurrentTurningSpeed = turningSpeed;



      // convert chassis speeds to individual module states; later to switch to velocity
      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
      // set state to each wheel


      swerveSubsystem.setModuleStates(moduleStates, true);

        

    }

    
    @Override
    public void end(boolean interrupted) {

      
   
    
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
