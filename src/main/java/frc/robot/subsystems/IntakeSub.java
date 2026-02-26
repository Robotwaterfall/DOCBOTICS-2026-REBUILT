package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort);

    /**
     * Description: Sets the intake motor's power
     * Pre-Condition: Motor is initialized (Power is between -1 and 1?)
     * Post-Condition: The motor's power is set
     * @param power The desired power for the motor
     */
    public void setMotorPower(double power) {
        intakeMotor.set(power);
    }

    /**
     * Description: Stops the motor
     * Pre-Condition: Motor must be initalized
     * Post-Condition: Motor is stopped
     */
    public void stopMotor() {
        intakeMotor.stopMotor();
    }


    @Override
    public String toString() {
        String str = "";

        str += "Intake Roller Information";
        str += "\nMotor Power: " + this.intakeMotor.getDutyCycle().getValueAsDouble();
        if (this.intakeMotor.getDutyCycle().getValueAsDouble() != 0)
            str += "\nCurrently: " + (this.intakeMotor.getDutyCycle().getValueAsDouble() > 0 ? "INTAKING" : "OUTAKING");

        return str;
    }

    // Getter for debugging 
    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }

}
