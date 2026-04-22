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
    public static final double headingToleranceDeg = 1.5; 
    public static final double kTurning = 0.2; 

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
      public static final double autoOrientKp = 0.015; 
      public static final double autoOrientSpeed = 0.6;
      public static final double autoOrientToleranceDegrees = 0.8;

    }
  }

  public static final class LimelightConstants {

    public static final String LimelightFront = "limelight-track";
    public static final String LimelightBackLeft = "limelight-left"; 

  }

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 2;

      public static final int kDriveGyroResetButtonIdx = 10;
    
      public static final int kL3Button = 11; //L3 Button
      public static final int kR3Button = 12;

      public static final int kDriverFieldOrientedButtonIdx = 9;

      public static final int kR2TriggerButton = 8;
      public static final int kL2TriggerButton = 7;

      public static final int kR1Button = 6;
      public static final int kL1Button = 5;

      public static final double kRumblePwr = 0.8;

      public static final int kDpadUP = 0;
      public static final int kDpadDOWN = 180;
      public static final int kDpadRIGHT = 90;
      public static final int kDpadLEFT = 270;
      public static final int kDpadRIGHTDOWN = 135;

      public static final int kTouchPadButton = 14;

      public static final int kSquareButton = 1;

      public static final int kPsButton = 13;

      public static final double kDeadband = 0.1;

  }
  public static final class IntakeRollerConstants{
    public static final int kIntakeMotorPort = 3; 

    public static final double kIntakePower = -0.85;
    public static final double kOuttakePower = 0.85;

  }
  
  public static final class PathPlannerConstants {
    public static final double kTranslationalKp = 5.0;
    public static final double kTranslationalKi = 0.0;
    public static final double kTranslationalKd = 0.0;

    public static final double kRotationalKp = 5.0;
    public static final double kRotationalKi = 0.0;
    public static final double kRotationalKd = 0.0;
    
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

    public static double intakePitcherWaitTimeSec = 0.5; 

    public static double intakePitcherFlutterDegrees = 40;

    public static final IdleMode pitcherIdleMode = IdleMode.kCoast;

    public static final double pitcherMaxSpeed = 0.3;
  }

   public static final class LEDConstants {
    public static final int BLINKIN_PWM_PORT = 0;
    public static final int BLINKIN2_PWM_PORT = 1;

    public static final double BLINKIN_PATTERN_RED_POS = -0.25;
    public static final double BLINKIN_PATTERN_BLUE_POS = -0.23;
    public static final double BLINKIN_PATTERN_ORANGE_BLINK_POS = 0.65;
    public static final double BLINKIN_PATTERN_RAINBOW_POS = -0.99; 
    public static final double BLINKIN_PATTERN_DOC_POS = -0.99; 
    public static final double BLINKIN_PATTERN_SHIFT = -0.07;
    public static final double HUB_SHIFT_WARNING_TIME = 3; // in seconds
  }

  public static final class ShooterConstants {
    public static final int kShooterLeadMotorId = 24;
    public static final int kShooterFollowerRightId = 23;
    public static final int kShooterFollowerLeftId = 22; 

    public static final int kIndexMotorId = 25; 

    public static final double kWheelDiameterInches = 4.0; // 4" wheel
    public static final double kGearRatio = 1.0;

    public static final double kShooterKP = 0.67; 
    public static final double kShooterKi = 0.45; 
    public static final double kShooterKd = 0; 
    public static final double kShooterKs = 0; 
    public static final double kShooterKv = 12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond);

    public static final double kIndexSpeed = 0.95;
    public static final double kReverseIndexSpeed = -0.95; 

    public static final double kWarmupVelocityFPS = 50;

    public static final double kJuggleVelocityFPS = 10;

    public static final double kHeightOfHubInches = 72; //height of the hub where fuel can enter

    public static final double shooterTolerance = 0.8; 

    public static final double defaultShooterVelocityPlusPerPress = 2.0; 

    public static final double shooterVelocityPlusPerPress = 2.0; 

    public static final double kShooterHeightInches = 0; //TODO: measure from floor to center of ball in shooter.
    
    public static final double shooterEfficiency = 0.8;
    
    public static final double kMaxShooterVelocityFPS  = 110;
    public static final double kMaxShooterRPM  = kMaxShooterVelocityFPS * 60 / (Math.PI * ShooterConstants.kWheelDiameterInches); 

    public static final double kShooterFixedAngle  = 60; 

     public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
  
    
  }
    public static final class ConveyorConstant {
    public static final int kConveyorMotorPort = 7;
    public static final double conveyorPower = 0.6; 
    public static final double reverseConveyorPower = -0.6; 
 

  }

  public static final class Pose2DConstants{
    
  // Field geometry in METERS (FRC coord: origin bottom-right(DRIVER PRESPECTIVE) BLUE alliance wall)
  public static final double BLUE_HUB_POSE_X_M = 4.63; // blue hub center
  public static final double BLUE_HUB_POSE_Y_M = 4.03; // blue hub center (alliance depth)
  
  // Full field
  public static final double FIELD_X_MIN_M = 0.0;
  public static final double FIELD_X_MAX_M = 17.55648;
  
  public static final double FIELD_Y_MIN_M = 0.0;
  public static final double FIELD_Y_MAX_M = 8.042656;

  // Blue alliance zone bounds (rectangle: full width x alliance depth)
  // Measured not as FIRST does (after bump), but by middle of bump.
  // This allows the robot to "be in zone" while shooting from trench
  public static final double BLUE_ZONE_X_MIN_M = 0.0;
  public static final double BLUE_ZONE_X_MAX_M = 4.625594;

  public static final double BLUE_ZONE_Y_MIN_M = 0.0;
  public static final double BLUE_ZONE_Y_MAX_M = FIELD_Y_MAX_M;

  // Alliance zone poses (left/right corners, bottom)
  // These are the points we aim at when feeding
  public static final double BLUE_ZONE_MIDPOINT_X_M = BLUE_ZONE_X_MAX_M / 2;
  
  public static final double BLUE_ZONE_LEFT_MIDPOINT_Y_M = 0.75 * FIELD_Y_MAX_M;

  public static final double BLUE_ZONE_RIGHT_MIDPOINT_Y_M = 0.25 * FIELD_Y_MAX_M;

  // Half field (left half: vertical midline, full width x 0 to half length)
  public static final double HALF_FIELD_Y_M = FIELD_Y_MAX_M / 2;

  // Neutral zone / wasteland (front rectangle: 0 to neutral depth x full field)
  public static final double NEUTRAL_ZONE_X_MIN_M = BLUE_ZONE_X_MAX_M; // Starts at end of blue zone
  public static final double NEUTRAL_ZONE_X_MAX_M = BLUE_ZONE_X_MAX_M + 7.2898;
  public static final double NEUTRAL_ZONE_Y_MIN_M = 0.0;
  public static final double NEUTRAL_ZONE_Y_MAX_M = FIELD_Y_MAX_M;

  }
  
}