
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePitcherConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakePitcherSub extends SubsystemBase {
    /**Motor that rotates the intake. */
    private TalonFX intakePitcherMotor = new TalonFX(IntakePitcherConstants.kIntakePitcherMotorPort);

    /**PID controller to rotate the intake to an angular setpoint */
    private PhoenixPIDController intakePitchController = new PhoenixPIDController(
            IntakePitcherConstants.intakePitcher_kP,
            IntakePitcherConstants.intakePitcher_kI,
            IntakePitcherConstants.intakePitcher_kD);
            
    /**angular set point of the pitcher in degrees. */
    private double pitcherSetpoint_degrees = 0;

    public IntakePitcherSub() {
        intakePitchController.enableContinuousInput(0, 360);
    }


    /**
     * @return the motor that pitches the intake.
     */
    public TalonFX getIntakePitcherMotor() {
        return intakePitcherMotor;
    }

    /** @return the PID controller of the intake pitcher */
    public PhoenixPIDController getIntakePitchController() {
        return intakePitchController;
    }
    
    /**Sets the angular setpoint of the intake in degrees. */
    public void setIntakePitchSetpoint_degrees(double setpoint_degrees) {
        pitcherSetpoint_degrees = setpoint_degrees;
    }

    /**@return the angular setpoint of the intake in degrees. */
    public double getIntakePitchSetpoint_degrees() {
        return pitcherSetpoint_degrees;
    }

}