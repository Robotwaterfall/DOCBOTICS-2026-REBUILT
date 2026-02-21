
package frc.robot.subsystems;


import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.config.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;







public class SwerveSub extends SubsystemBase {
    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);




    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
            
    private final SwerveModuleState[] mySwerveStates = new SwerveModuleState[]{ // used for debugging to Adavantage Scope
        frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
    };


    private final SwerveModule swerveModules[] = new SwerveModule[]{
        frontLeft,frontRight,
        backLeft, backRight
    };

    private double limeLightTX = 0;


    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, 
        getRotation2d(), 
        getModulePositionsAuto(),
        new Pose2d() );


    private RobotConfig config;
    
    // CAN ID 1 on the rio CAN bus – change to your real ID / bus name
    private final Pigeon2 gyro = new Pigeon2(1, "rio");


    private final Field2d m_Field = new Field2d();
    
    //new AHRS(SerialPort.Port.kUSB1);




    public SwerveSub(){



 new Thread(() -> {  /// try catch function is a fancy if else statement
        try{              // it tries to run a thread of resseting the gryo but if it exception e happens it stops 
            Thread.sleep(1000);
        }catch (Exception e){
        }
        }).start();

        zeroHeading();
        
            // Load the RobotConfig from the GUI settings. You should probably
            // store this in your Constants file
            
            try{
            config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
    }
        
          AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                
            ),
            config,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", m_Field);

    
    }
    @Override
    public void periodic(){

        poseEstimator.update(getRotation2d(),  getModulePositionsAuto()
        );

        boolean doRejectUpdate = LimelightHelpers.getTV("limelight3"); // TODO: Change limelight
        if(doRejectUpdate){
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight3");


            if(mt1.tagCount > 0){
                poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }

        Logger.recordOutput("RobotPose", poseEstimator.getEstimatedPosition());

        SmartDashboard.putNumber("robot Heading", getHeading());
        SmartDashboard.putString("robot location", getPose().getTranslation().toString());


        SwerveModulePosition[] debugModulePosition = getModulePositionsAuto();
        for(int i = 0; i <= 3; ++i){
            SmartDashboard.putString("SwerveModulePostions [" + i + "]" , "distance : " + debugModulePosition[i].distanceMeters
            + "Speeds : " + debugModulePosition[i].angle);

        Logger.recordOutput("pose2d", getPose());

        SmartDashboard.putNumber("SwerveModuleTurningPostions [" + 1 + "]" ,  frontLeft.getTurningPosition());
        SmartDashboard.putNumber("SwerveModuleTurningPostions [" + 2 + "]" ,  frontRight.getTurningPosition());
        SmartDashboard.putNumber("SwerveModuleTurningPostions [" + 3 + "]" ,  backLeft.getTurningPosition());
        SmartDashboard.putNumber("SwerveModuleTurningPostions [" + 4 + "]" ,  backRight.getTurningPosition());




}
        Logger.recordOutput("heading",getHeading());
      




        frontLeft.sendToDashboard();
        frontRight.sendToDashboard();
        backLeft.sendToDashboard();
        backRight.sendToDashboard();
    }
    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    } 
    public void resetPose(Pose2d pose){
        poseEstimator.resetPosition(getRotation2d(), getModulePositionsAuto() , pose);
    }
     public ChassisSpeeds getSpeeds() {
         return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()); 
    }


    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates, true);
    }




    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isDeadband){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    // proportaionally decreases the change the speeds so driver always had control of robot
        frontRight.setDesiredState(desiredStates[0], isDeadband);        
        frontLeft.setDesiredState(desiredStates[1] , isDeadband); //1 
        backRight.setDesiredState(desiredStates[2], isDeadband); //2                     
        backLeft.setDesiredState(desiredStates[3],isDeadband); // 3



        //ouputs to Adavantage Log

        // log desired states is an array that orders the desired states in the order 
        // Advantage Log wants ( FL,FR, BL, BR )
        SwerveModuleState[] LogDesiredStates = new SwerveModuleState[]{desiredStates[1], desiredStates[0],
         desiredStates[3], desiredStates[2]};


        Logger.recordOutput("CurrentStates", mySwerveStates);
        Logger.recordOutput("DesiredStates",LogDesiredStates);
    
    }

// get positions
public SwerveModulePosition[] getModulePositionsAuto() { // not updating
    SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getSwerveModulePosition();
    }
    return positions;
  }


    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        double angleDeg = gyro.getRotation2d().getDegrees();
        return Math.IEEEremainder(-angleDeg, 360); //puts the value between 0 and 360 because gryo is naturally continous
    }

    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    } // converts into Rotation2d

    public void resetSwerveModules(){
        
    }


      public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
          states[i] = swerveModules[i].getState();
        }
        return states;
      }



    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();

    }




    public double orientToTarget(){
        if(LimelightHelpers.getTV(LimelightConstants.Limelight2)){
        limeLightTX = LimelightHelpers.getTX(LimelightConstants.Limelight2); 
        }
        double targetingAngularVelocity = 
        limeLightTX * 
        Constants.DriveConstants.autoTargetConstants.autoOrientKp;
    

        // convert to radians per second for our drive method
        targetingAngularVelocity *= DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        //if there were apply a really small power output to the 
        // turning motor, stop applying power
        if (Math.abs(targetingAngularVelocity)  <= 0.001){
            targetingAngularVelocity = 0;
        }

        return targetingAngularVelocity;

    }



}
