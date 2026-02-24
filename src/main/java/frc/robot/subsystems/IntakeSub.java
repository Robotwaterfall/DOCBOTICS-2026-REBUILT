package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.kIntakeMotorPort);

    public void setMotorPower(double power) {
        intakeMotor.set(power);
    }

    // 
    public TalonFX getIntakeMotor() {
        return intakeMotor;
    }


    /**
     * Description: Stops the motor
     * Pre-Condition: Motor must be initalized
     * Post-Condition: Motor is stopped
     */
    public void stopMotor() {
        intakeMotor.stopMotor();
    }

}
