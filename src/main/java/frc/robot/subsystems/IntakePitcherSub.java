package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.IntakePitcherConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakePitcherSub extends SubsystemBase {
    private final SparkMax intakePitcherMotor;
    private final SparkMaxConfig intakePitcherMotorConfig;
    private final PIDController intakePitchController;
    
    private double targetAngle = 0;
    private double currentAngle = 0;

    public IntakePitcherSub() {
        intakePitcherMotor = new SparkMax(IntakePitcherConstants.kIntakePitcherMotorPort, MotorType.kBrushless);
        intakePitcherMotorConfig = new SparkMaxConfig();
        intakePitchController = new PIDController(
            IntakePitcherConstants.intakePitcher_kP,
            IntakePitcherConstants.intakePitcher_kI,
            IntakePitcherConstants.intakePitcher_kD);

        // Encoder setup - CONVERTED TO DEGREES
        intakePitcherMotorConfig.encoder
            .positionConversionFactor(IntakePitcherConstants.kDegreesPerMotorRotation)
            .velocityConversionFactor(IntakePitcherConstants.kDegreesPerMotorRotation);

        // Motor behavior
        intakePitcherMotorConfig.idleMode(Constants.IntakePitcherConstants.pitcherIdleMode);
        intakePitcherMotorConfig.smartCurrentLimit(40);
        
        intakePitcherMotorConfig.inverted(true);

        intakePitcherMotor.configure(intakePitcherMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        intakePitchController.setTolerance(IntakePitcherConstants.intakePitcherToleranceDegrees);
    }
    
    @Override
    public void periodic() {
        // CRITICAL: Run PID every loop!
        currentAngle = intakePitcherMotor.getEncoder().getPosition();
        double output = intakePitchController.calculate(currentAngle, targetAngle);
        output = MathUtil.clamp(output, -Constants.IntakePitcherConstants.pitcherMaxSpeed, 
            Constants.IntakePitcherConstants.pitcherMaxSpeed);
        intakePitcherMotor.set(output);
        
        // Debug
        SmartDashboard.putNumber("Pitcher Error", intakePitchController.getPositionError());
        SmartDashboard.putBoolean("Pitcher At Target", intakePitchController.atSetpoint());
    }

    public void setPitcherAngle(double degrees) {
        targetAngle = degrees;
    }

    public boolean isPitcherAtSetpoint(){
        return intakePitchController.atSetpoint();
    }

    public void stopMotor() {
        intakePitcherMotor.stopMotor();
    }


    // Getters
    public SparkMax getIntakePitcherMotor() { return intakePitcherMotor; }
    public PIDController getIntakePitchController() { return intakePitchController; }
    public double getCurrentAngle() { return currentAngle; }
    public double getTargetAngle() { return targetAngle; }
}
