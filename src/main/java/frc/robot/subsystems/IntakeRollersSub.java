package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRollersSub extends SubsystemBase {
    private final TalonFX intakeRollerMotor = new TalonFX(IntakeRollerConstants.kIntakeMotorPort);

    public IntakeRollersSub() {
    }

    /** Stops the motor with neutral output */
    public void stopMotor() {
        intakeRollerMotor.stopMotor();
    }

    /** Legacy voltage control (use sparingly for open-loop testing) */
    public void setMotorPower(double percentOutput) {
        intakeRollerMotor.set(percentOutput);
    }

    // Getters for debugging/tuning
    public TalonFX getIntakeRollerMotor() {
        return intakeRollerMotor;
    }


    @Override
    public void periodic() {

    }
}
