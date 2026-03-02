// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurningMotorGearRatio = 1 / 12.8;

    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    public static final double kTurning = 0.3; // TODO: Tune this kp for turning

  }

  public static final class DriveConstants {

      public static final double kTrackWidth = Units.inchesToMeters(19.75); //changed
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(28.6); //changed 2025 01 25
      // Distance between front and back wheels
      public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(kWheelBase / 2, kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kFrontLeftDriveMotorPort = 1; //TODO CHANGE ALL SWERVE IDS
    public static final int kBackLeftDriveMotorPort = 2;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kBackRightDriveMotorPort = 4;

    public static final int kFrontLeftTurningMotorPort = 5;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final int kBackRightTurningMotorPort = 8;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kBackRightDriveEncoderReversed = true;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 10;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
    public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.3481 * 2 * Math.PI;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.2556 * 2 * Math.PI;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -0.3181 * 2 * Math.PI;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.094971 * 2 * Math.PI;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond  / 1.75;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

      public static final int kImuIdPort = 13; //TODO

    public static class autoTargetConstants {
      public static final double autoOrientKp = 0.0035;

      public static final double autoLockKp = 0;

    }
  }

  public static final class LimelightConstants {

    public static final String Limelight2 = "limelight2+";
    public static final String Limelight3 = "limelight3";

    public static final double limelightMountAngleDegrees = 60.0; //TODO
    public static final double limelightLensHeightInches = 20.0; //TODO
    public static final double goalHeightInches = 49.5;

  }

    public static final class OIConstants {

      public static final int kDriverControllerPort = 0;

      public static final int kDriverXAxis = 0;
      public static final int kDriverYAxis = 1;
      public static final int kDriverRotAxis = 4;

      public static final int kDriveGyroResetButtonIdx = 7;
    
      public static final int kSlowModeIdx = 4;

      public static final int kDriverFieldOrientedButtonIdx = 5;

      public static final int kLockWheelsButton = 9;



      public static final double kDeadband = 0.5;

  }
  public static final class IntakeConstants{
    public static final int kIntakeMotorPort = 14; //TODO:: SET LATER
    public static final double kIntakeMotorPower = 1.0; //TODO
    public static final double kOutakeMotorPower = -1.0; //TODO

  }

  public static final class IntakePitcherConstants {
    public static final int kIntakePitcherMotorPort = 15; //TODO:: SET LATER
    public static final double kDegreesPerMotorRotation = 360 / (9 * 48/20 * 48/20 * 24/12); // Degrees / gear ratio gives degrees to motor revolutions

    public static final double kMinPitchDegrees = 0;
    public static final double kMaxPitchDegrees = 90; //TODO

    public static final double kPitcherIn = 0; // Setpoint positions
    public static final double kPitcherOut = 90; //TODO

    public static double intakePitcher_kP = 0;
    public static double intakePitcher_kI = 0;
    public static double intakePitcher_kD = 0;
  }

  public static final class ShooterConstants {
    public static final int kShooterLeadMotorPort = 16; //TODO:: SET LATER
    public static final int kShooterFollower_1_port = 17; //TODO:: SET LATER
    public static final int kShooterFollower_2_port = 18; //TODO:: SET LATER

    public static final int kIndexMotor_Port = 19; //TODO: set later

    public static final double kWheelDiameterMeters = 0.1016; // 4" wheel
    public static final double kGearRatio = 1.0;

    public static final double kShooterKP = 0.1; //TODO
    public static final double kShooterKi = 0.1; //TODO
    public static final double kShooterKd = 0.1; //TODO

    public static final double indexSpeed = 0.4; //TODO


    //PHYSICS CONSTANTS
    public static final double kGravity = 9.8;
    public static final double heightOfGoalMeters = 1.82; //height of the hub where fuel can enter

    public static final double shooterTolerance = 0; //TODO
  
    
  }
    public static final class ConveyorConstant {
    public static final int kConveyorMotorPort = 20; //TODO:: SET LATER
    public static final double conveyorPower = 0.6;
 

  }

  public static final class HoodConstants{
    public static final int kHoodId = 21; //TODO: change can id for hood

    public static final double kMinAngleDeg = 0.0; //TODO:tune
    public static final double kMaxAngleDeg = 180.0;
    public static final int kMinPulseUs = 500;  //TODO:tune
    public static final int kCenterPulseUs = 1500;
    public static final int kMaxPulseUs = 2500;  
    
  }

  public static final class unitConversions{

    public static final double inchesToMeters = 0.0254; //multiply with inches to get meters
  }

  public static final class autoConstants{

    public static final double timeElapsedShootingSecounds = 8; //TODO
    public static final double timeBetweenPitcherInAndOut = 1; //TODO
  }
}
