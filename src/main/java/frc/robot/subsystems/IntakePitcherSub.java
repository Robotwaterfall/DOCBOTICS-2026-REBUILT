package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakePitcherConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakePitcherSub extends SubsystemBase {
    /**Motor that rotates the intake. */
    private SparkMax intakePitcherMotor = new SparkMax(IntakePitcherConstants.kIntakePitcherMotorPort, MotorType.kBrushless);

    /**Motor controller configuratiion of the intake pitcher motor. */
    private SparkMaxConfig intakePitcherMotorConfig = new SparkMaxConfig();
    /**PID controller to rotate the intake to an angular setpoint */
    private PIDController intakePitchController = new PIDController(
            IntakePitcherConstants.intakePitcher_kP,
            IntakePitcherConstants.intakePitcher_kI,
            IntakePitcherConstants.intakePitcher_kD);
            
    /**angular set point of the pitcher in degrees. */
    private double targetAngle = 0;
    private double currentAngle = 0;


    public IntakePitcherSub() {
     
      intakePitcherMotorConfig = new SparkMaxConfig();

      // Encoder setup
      intakePitcherMotorConfig.encoder
        .positionConversionFactor(IntakePitcherConstants.kDegreesPerMotorRotation)
        .velocityConversionFactor(IntakePitcherConstants.kDegreesPerMotorRotation);

      // Soft limits (in *converted units*, NOT raw rotations)
      intakePitcherMotorConfig.softLimit
        .forwardSoftLimit(IntakePitcherConstants.kMaxPitchDegrees)
        .reverseSoftLimit(IntakePitcherConstants.kMinPitchDegrees)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

      // Motor behavior
      intakePitcherMotorConfig.idleMode(Constants.IntakePitcherConstants.pitcherIdleMode);
      intakePitcherMotorConfig.smartCurrentLimit(40);

      // Apply config
      intakePitcherMotor.configure(intakePitcherMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      // PID controller setup
      intakePitchController.setTolerance(IntakePitcherConstants.intakePitcherToleranceDegrees); // degrees

    }

    
    /**
     * Description: Rotates the pitcher to a specified degree position
     * Pre-Condition: All objects and hardware must be declared and intialized
     * Post-Condition: Pitcher has rotated to desired position
     * @param degrees The desired degrees rotated to
     */
    public void movePicherToSetpoint(double degrees) {

        // Positions
        targetAngle = degrees;
        currentAngle = intakePitcherMotor.getEncoder().getPosition() * IntakePitcherConstants.kDegreesPerMotorRotation;

        // PID
        double output = intakePitchController.calculate(currentAngle, targetAngle);
        output = MathUtil.clamp(output, -Constants.IntakePitcherConstants.pitcherMaxSpeed, 
            Constants.IntakePitcherConstants.pitcherMaxSpeed);

        // Setting power
        intakePitcherMotor.set(output);

    }


    /**
     * Description: Stops the motor
     * Pre-Condition: Motor must be initalized
     * Post-Condition: Motor is stopped
     */
    public void stopMotor() {
        intakePitcherMotor.stopMotor();
    }


    @Override
    public String toString() {
        String str = "";

        str += "Intake Pitcher Information";
        str += "\nCurrent Position: " + currentAngle + " degrees.";
        str += "\nTarget Position: " + targetAngle + " degrees.";
        str += "\nMotor Power: " + this.intakePitcherMotor.getAppliedOutput();

        return str;
    }

    // debug getters and whatnot
    /**
     * @return the motor that pitches the intake.
     */
    public SparkMax getIntakePitcherMotor() {
        return intakePitcherMotor;
    }

    /** @return the PID controller of the intake pitcher */
    public PIDController getIntakePitchController() {
        return intakePitchController;
    }
    /**Sets the angular setpoint of the intake in degrees. */
    public void setIntakePitchSetpoint_degrees(double setpoint_degrees) {
        targetAngle = setpoint_degrees;
    }
    /**@return the angular setpoint of the intake in degrees. */
    public double getIntakePitchSetpoint_degrees() {
        return targetAngle;
    }

}