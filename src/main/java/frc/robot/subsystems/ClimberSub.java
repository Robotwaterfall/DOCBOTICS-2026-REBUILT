package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSub extends SubsystemBase{

    private SparkMax primaryClimberMotor = new SparkMax(ClimberConstants.kClimberPrimaryMotorId, MotorType.kBrushless); //right (back of robot)
    private SparkMax secondaryClimberMotor = new SparkMax(ClimberConstants.kClimberSecoundaryMotorId, MotorType.kBrushless); //left (back of robot)

    private SparkMaxConfig primaryClimberMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig secondaryClimberMotorConfig = new SparkMaxConfig();

    private double climberSetpoint;

    private PIDController climberController = new PIDController(
        ClimberConstants.Kp, 
        ClimberConstants.Ki, 
        ClimberConstants.Kd);

    public ClimberSub(){

        secondaryClimberMotorConfig.follow(
            ClimberConstants.kClimberPrimaryMotorId,true
        );

        primaryClimberMotorConfig.encoder.positionConversionFactor(ClimberConstants.elevatorMotorRotationToMeters);

        primaryClimberMotor.configure(primaryClimberMotorConfig, 
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SparkMax getPrimaryClimberMotor(){
        return primaryClimberMotor;
    }

    public SparkMax getSecondaryClimberMotor(){
        return secondaryClimberMotor;
    }

    public double getPrimaryClimberPosition(){
        return primaryClimberMotor.getEncoder().getPosition();
    }

    public PIDController getClimberController(){
        return climberController;
    }

    public void setClimberSetpoint(double setpoint){
        climberSetpoint = setpoint;
    }
    
    public double getClimberSetpoint(){
        return climberSetpoint;
    }

    public void resetElevatorEncoders(){
        primaryClimberMotor.getEncoder().setPosition(0);
    }

}
