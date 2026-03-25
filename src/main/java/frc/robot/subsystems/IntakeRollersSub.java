package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;;

public class IntakeRollersSub extends SubsystemBase {

    private final SparkMax intakeRollerMotor = new SparkMax(IntakeRollerConstants.kIntakeMotorPort, MotorType.kBrushless);
    private final SparkMaxConfig intakeRollerMotorConfig = new SparkMaxConfig();

    public IntakeRollersSub(){

        intakeRollerMotorConfig.inverted(true);
        intakeRollerMotorConfig.idleMode(IdleMode.kCoast);

        intakeRollerMotor.configure(intakeRollerMotorConfig, ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    /**
     * Description: Sets the intake motor's power
     * Pre-Condition: Motor is initialized (Power is between -1 and 1)
     * Post-Condition: The motor's power is set
     * @param power The desired power for the motor
     */
    public void setMotorPower(double power) {
        intakeRollerMotor.set(power);
    }

    /**
     * Description: Stops the motor
     * Pre-Condition: Motor must be initalized
     * Post-Condition: Motor is stopped
     */
    public void stopMotor() {
        intakeRollerMotor.stopMotor();
    }


    // Getter for debugging 
    public SparkMax getIntakeRollerMotor() {
        return intakeRollerMotor;
    }

    public boolean isIntakeRollersRunning() {
        return intakeRollerMotor.get() != 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isIntakeRollersRunning", isIntakeRollersRunning());
    }

}
