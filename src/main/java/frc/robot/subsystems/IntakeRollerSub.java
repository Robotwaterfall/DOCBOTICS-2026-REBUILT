package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;;

public class IntakeSub extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

    /**
     * Description: Sets the intake motor's power
     * Pre-Condition: Motor is initialized (Power is between -1 and 1)
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
        double power = this.intakeMotor.getAppliedOutput();

        str += "Intake Roller Information";
        str += "\nMotor Power: " + power;
        if (power != 0)
            str += "\nCurrently: " + (power > 0 ? "INTAKING" : "OUTAKING");

        return str;
    }

    // Getter for debugging 
    public SparkMax getIntakeMotor() {
        return intakeMotor;
    }

}
