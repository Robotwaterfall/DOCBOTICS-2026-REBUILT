package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

    private final SparkMax turningMotor;
    private final SparkMaxConfig turningMotorConfig = new SparkMaxConfig();

    private final PIDController turningController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(
        int driveMotorId, 
    int turningMotorId, 
        boolean driveMotorReversed,
    boolean turningMotorReversed, 
        int absoluteEncoderId, 
    double absoluteEncoderOffset, 
        boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

        driveMotorConfig
        .inverted(driveMotorReversed);

        turningMotorConfig
        .inverted(turningMotorReversed);

        driveMotorConfig.encoder
        .positionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(SwerveModuleConstants.kDriveEncoderRPM2MeterPerSec);

        turningMotorConfig.encoder
        .positionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningController = new PIDController(SwerveModuleConstants.kTurning,0,0);

        turningController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoder();

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public double getDrivePosition(){
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPosition(){
        return ((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI) - absoluteEncoderOffsetRad);
    }

    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad(){

        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble()); // reads the percent of how much rotation we are reading
        angle *= 2.0 * Math.PI; //convert to radians
        angle -= absoluteEncoderOffsetRad;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0); //gives the Encoder values on if the encoder is reversed
    } 

    public void resetEncoder(){
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, boolean isDeadband){
        if(Math.abs(state.speedMetersPerSecond) < 0.001 && isDeadband){ // were not moving do not reset the motors
            
            stop();
            return;
        }
        
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningController.calculate(getTurningPosition(), state.angle.getRadians()));
        
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
        turningController.setSetpoint(0);
    }

    public void sendToDashboard(){
        //insert live data to send dashboard here

        SmartDashboard.putNumber("Drive[" + absoluteEncoder.getDeviceID() + "] output", driveMotor.getAppliedOutput());


        SmartDashboard.putNumber("Turning[" + absoluteEncoder.getDeviceID() + "] output", turningMotor.getAppliedOutput());

        SmartDashboard.putNumber("DrivePos[" + absoluteEncoder.getDeviceID() + "]", getDriveVelocity());



        SmartDashboard.putNumber("TurningPos[" + absoluteEncoder.getDeviceID() + "]", getTurningPosition());

        SmartDashboard.putNumber("AbsPos[" + absoluteEncoder.getDeviceID() + "] ", absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }


}
