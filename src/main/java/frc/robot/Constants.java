// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

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

    public static final double kTurning = 0.3; 

  }

  public static final class LockOnPoseConstants {

    public static final double kMax_Rotational_Speed = 0.8;
    public static final double headingToleranceDeg = 2.0; 
    public static final double kTurning = 0.6; 

  } 

  public static final class DriveConstants {

      public static final double kTrackWidth = Units.inchesToMeters(20.594); 
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(25.50); 
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

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond  / 1.0;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final int kImuIdPort = 21;

    public static final double kSlowModeFactor = 0.35;
    

    public static class autoTargetConstants {
      public static final double autoOrientKp = 0.02; 
      public static final double autoOrientSpeed = 0.5;

    }
  }

  public static final class LimelightConstants {

    public static final String LimelightFront = "limelight-track";
    public static final String LimelightBackLeft = "limelight-left"; 
    public static final String LimelightBackRight = "limelightbackright"; //TODO

  }

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 2;

      public static final int kDriveGyroResetButtonIdx = 10;
    
      public static final int kSlowModeIdx = 11;

      public static final int kDriverFieldOrientedButtonIdx = 9;

      public static final int kShootingRoutineButton = 8;
      public static final int kPrepareShotButton = 7;

      public static final int kIntakeButton = 6;
      public static final int kOuttakeButton = 5;

      public static final double kRumblePwr = 0.8;

      public static final int kDpadUP = 0;
      public static final int kDpadDOWN = 180;
      public static final int kDpadRIGHT = 90;
      public static final int kDpadLEFT = 270;
      public static final int kDpadRIGHTDOWN = 135;

      public static final int kTouchPadButton = 14;

      public static final int kPsButton = 13;

      public static final double kDeadband = 0.5;

  }
  public static final class IntakeRollerConstants{
    public static final int kIntakeMotorPort = 3; 
    
    public static final double kIntakeRollersKp = 0.5; //TODO
    public static final double kIntakeRollersKi = 0; //TODO
    public static final double kIntakeRollersKd = 0; //TODO
    public static final double kIntakeRollersKs = 0; //TODO
    public static final double kIntakeRollersKv = 0; //TODO

    public static final double kIntakeVelocityRPS = -0.8;
    public static final double kOuttakeVelocityRPS = 0.8;

  }

  public static final class IntakePitcherConstants {
    public static final int kIntakePitcherMotorPort = 2; 
    public static final double kDegreesPerMotorRotation = 360 / (9 * 48/20 * 48/20 * 24/12); // Degrees / gear ratio gives degrees to motor revolutions

    public static final double kMinPitchDegrees = 0; 
    public static final double kMaxPitchDegrees = 105; 

    public static final double kPitcherInDegrees = 0; // Setpoint positions
    public static final double kPitcherOutDegrees = 103; 

    public static double intakePitcher_kP = 0.1; 
    public static double intakePitcher_kI = 0; 
    public static double intakePitcher_kD = 0; 

    public static double intakePitcherToleranceDegrees = 1;

    public static double intakePitcherWaitTimeSec = 0.8; 

    public static double intakePitcherFlutterDegrees = 95;

    public static final IdleMode pitcherIdleMode = IdleMode.kCoast;

    public static final double pitcherMaxSpeed = 0.25;
  }

  public static final class ShooterConstants {
    public static final int kShooterLeadMotorId = 24;
    public static final int kShooterFollowerRightId = 23;
    public static final int kShooterFollowerLeftId = 22; 

    public static final int kShooterVelocityFps = 45; //TODO: GET RID FOR TESTING

    public static final int kIndexMotorId = 25; 

    public static final double kWheelDiameterInches = 4.0; // 4" wheel
    public static final double kGearRatio = 1.0;

    public static final double kShooterKP = 0.67; 
    public static final double kShooterKi = 0.45; 
    public static final double kShooterKd = 0; 
    public static final double kShooterKs = 0; 
    public static final double kShooterKv = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond); //TODO

    public static final double kIndexSpeed = 1;
    public static final double kReverseIndexSpeedWhileIntaking = -0.9;
    public static final double kReverseIndexSpeed = -1; 

    public static final double kWarmupVelocityFPS = 50;

    public static final double kJuggleVelocityFPS = 10;

    public static final double kHeightOfHubInches = 72; //height of the hub where fuel can enter

    public static final double shooterTolerance = 2.0; 

    public static final double defaultShooterVelocityPlusPerPress = 5.0; 

    public static final double shooterVelocityPlusPerPress = 5.0; 

     public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
  
    
  }
    public static final class ConveyorConstant {
    public static final int kConveyorMotorPort = 7;
    public static final double conveyorPower = 0.6; 
    public static final double reverseConveyorPower = -0.6; 
 

  }

  public static final class HoodConstants{
    public static final int kHoodId = 30; 

    public static final int kMinPulseUs = 1650;  //this is the robots limit of the Lin act
    public static final int kCenterPulseUs = 1750;
    public static final int kMaxPulseUs = 2000;
    
    public static final double maxExtensionInches = 5.5; //as far as we go out
    public static final double minExtensionInches = 3; //as far as we retract 

    public static final double minHoodAngleDeg = 43; 
    public static final double maxHoodAngleDeg = 63; 

    public static final double kHoodWarmUpDeg = 50; //warmp up position for hood to be in before shooting

    public static final double kHoodToleranceUs = 10; //TODO: tune this tolerance for being at the setpoint

    public static final double defaultHoodAnglePlusPerPress = 2; //TODO
    public static final double hoodAnglePlusPerPress = 2; //TODO
    
    //these are the constants for the cubic approximation to convert from angle to inches
    public static final double a0 = 8.15031546257;
    public static final double a1 = 0.0376843712267;
    public static final double a2 = -0.00289877526491;
    public static final double a3 = 1.49396327886e-05;
  }


  public static final class Pose2DConstants{
    
    public static final double xHubPose = 4.63; // blue hub center (meters), derived from AprilTag ring
    public static final double yHubPose = 4.03; // blue hub center (meters), derived from AprilTag ring

    public static final double ALLIANCE_ZONE_X_MIN_BLUEin = 0.0;
    public static final double ALLIANCE_ZONE_X_MAX_BLUEin = 182.11;
    public static final double ALLIANCE_ZONE_Y_MIN_BLUEin = 0.0;
    public static final double ALLIANCE_ZONE_Y_MAX_BLUEin = 317.68; 

    
  }

  public static final class GeorgianCollegeConstants{
    public static final int kCloseShotButton = 1;
    public static final int kFarShotButton = 2;
    public static final int kNeutralShotButton = 3;

    public static final double kShootFarVelocity = 100;
    public static final double kShootFarAngle = 45;

    public static final double kShootCloseVelocity = 45;
    public static final double kShootCloseAngle = 56;

  }
  
  public static final double telemetryUpdate = 0.1; //Update every 100ms
}