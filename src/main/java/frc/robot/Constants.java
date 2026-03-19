// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
   
  }

  public static class SwerveModuleConstants {

    public static final double kWheelDiameterInches = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterInches;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kTurning = 0.3; // TODO: Tune this kp for turning

  }

  public static final class LockOnPoseConstants {

    public static final double kMax_Rotational_Speed = 0.5;//TODO:find max lock on speed
    public static final double headingToleranceDeg = 2.0; //TODO:find tolerance 
    public static final double kTurning = 0.3; //TODO

  } 

  public static final class DriveConstants {

      public static final double kTrackWidth = Units.inchesToMeters(20.594); //TODO: measure track width and set here
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(25.50); //TODO: measure wheel base and set here
      // Distance between front and back wheels
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 15; 
    public static final int kBackLeftDriveMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 11;
    public static final int kBackRightDriveMotorPort = 14;

    public static final int kFrontLeftTurningMotorPort = 6;
    public static final int kBackLeftTurningMotorPort = 16;
    public static final int kFrontRightTurningMotorPort = 32;
    public static final int kBackRightTurningMotorPort = 12;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = true;
    public static final boolean kBackLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 29; 
    public static final int kBackLeftDriveAbsoluteEncoderPort = 26;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 27;
    public static final int kBackRightDriveAbsoluteEncoderPort = 28;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.422852 * 2 * Math.PI; 
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.162354 * 2 * Math.PI;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.083252 * 2 * Math.PI;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.341797 * 2 * Math.PI;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond  / 1.75;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final int kImuIdPort = 21;
    

    public static class autoTargetConstants {
      public static final double autoOrientKp = 0.0035;

      public static final double autoLockKp = 0; //TODO

    }
  }

  public static final class LimelightConstants {

    public static final String Limelight2 = "limelight-track";
    public static final String Limelight3 = "limelight3"; //TODO

    public static final double limelightMountAngleDegrees = 28.1; //TODO
    public static final double limelightLensHeightInches = 26.0; //TODO
  }

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 4;

      public static final int kDriveGyroResetButtonIdx = 10;
    
      public static final int kSlowModeIdx = 11;

      public static final int kDriverFieldOrientedButtonIdx = 9;

      public static final int kLockWheelsButton = 12;



      public static final double kDeadband = 0.5;

  }
  public static final class IntakeRollerConstants{
    public static final int kIntakeMotorPort = 3; 
    public static final double kIntakeMotorPower = -1.0; //reverse power to intake, positive power to outtake
    public static final double kOutakeMotorPower = 1.0; 

  }

  public static final class IntakePitcherConstants {
    public static final int kIntakePitcherMotorPort = 2; 
    public static final double kDegreesPerMotorRotation = 360 / (9 * 48/20 * 48/20 * 24/12); // Degrees / gear ratio gives degrees to motor revolutions

    public static final double kMinPitchDegrees = 0; //TODO: set min and max pitch degrees based on physical limits of the mechanism
    public static final double kMaxPitchDegrees = 100; //TODO

    public static final double kPitcherIn = 0; // Setpoint positions
    public static final double kPitcherOut = 90; //TODO

    public static double intakePitcher_kP = 0; //TODO
    public static double intakePitcher_kI = 0; //TODO
    public static double intakePitcher_kD = 0; //TODO

    public static double intakePitcherToleranceDegrees = 2; //TODO

    public static double intakePitcherWaitTimeSec = 0.8; //TODO

    public static final IdleMode pitcherIdleMode = IdleMode.kBrake;

    public static final double pitcherMaxSpeed = 0.5; //Has to be in between -1 and 1 //TODO
  }

  public static final class ShooterConstants {
    public static final int kShooterLeadMotorId = 24;
    public static final int kShooterFollowerRightId = 23;
    public static final int kShooterFollowerLeftId = 22; 

    public static final int kIndexMotorId = 25; 

    public static final double kWheelDiameterInches = 4.0; // 4" wheel
    public static final double kGearRatio = 1.0;

    public static final double kShooterKP = 0.1; //TODO
    public static final double kShooterKi = 0.1; //TODO
    public static final double kShooterKd = 0.1; //TODO
    public static final double kShooterKs = 0.1; //TODO
    public static final double kShooterKv = 0.1; //TODO

    public static final double kIndexSpeed = 1; 

    public static final double kWarmupVelocityFPS = 60; //TODO: find the warmup velocity in feet per second

    public static final double kHeightOfHubInches = 72; //height of the hub where fuel can enter

    public static final double shooterTolerance = 2.0; //TODO
  
    
  }
    public static final class ConveyorConstant {
    public static final int kConveyorMotorPort = 7;
    public static final double conveyorPower = 0.8; //TODO
 

  }

  public static final class HoodConstants{
    public static final int kHoodId = 30; 

    public static final double kDefaultAngleDeg = 60; //TODO: set default hood angle 
    public static final double kMinAngleDeg = 0.0; //TODO:tune
    public static final double kMaxAngleDeg = 180.0;
    public static final int kMinPulseUs = 1650;  //this is the hardware limit of the Lin act //TODO:tune
    public static final int kCenterPulseUs = 1750;
    public static final int kMaxPulseUs = 2000;
    
    public static final double maxExtensionInches = 5.5; //as far as we go out
    public static final double minExtensionInches = 3; //as far as we retract //TODO: tune
    
    //these are the constants for the cubic approximation to convert from angle to inches
    public static final double a0 = 8.15031546257;
    public static final double a1 = 0.0376843712267;
    public static final double a2 = -0.00289877526491;
    public static final double a3 = 1.49396327886e-05;
  }

  public static final class autoConstants{

    public static final double timeElapsedShootingSecounds = 8; //TODO
    public static final double timeBetweenPitcherInAndOut = 1; //TODO
  }

  public static final class Pose2DConstants{
    
    public static final double xHubPose = 8.23; //default blue
    public static final double yHubPose = 8.23; //default blue

    public static final double ALLIANCE_ZONE_X_MIN_BLUEin = 0.0;
    public static final double ALLIANCE_ZONE_X_MAX_BLUEin = 182.11;
    public static final double ALLIANCE_ZONE_Y_MIN_BLUEin = 0.0;
    public static final double ALLIANCE_ZONE_Y_MAX_BLUEin = 317.68; 

    
  }
  
  public static final double telemetryUpdate = 0.1; //Update every 100ms
}