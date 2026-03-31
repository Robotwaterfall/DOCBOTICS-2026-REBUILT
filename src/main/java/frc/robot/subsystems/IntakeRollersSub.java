package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRollersSub extends SubsystemBase {
    private final TalonFX intakeRollerMotor = new TalonFX(IntakeRollerConstants.kIntakeMotorPort);

    private final VelocityVoltage velocityReq = new VelocityVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeRollersSub() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        Slot0Configs slot0 = configs.Slot0;

        /* Velocity PID gains - tune these in Constants */
        slot0.kP = IntakeRollerConstants.kIntakeRollersKp;
        slot0.kI = IntakeRollerConstants.kIntakeRollersKi;
        slot0.kD = IntakeRollerConstants.kIntakeRollersKd;
        slot0.kS = IntakeRollerConstants.kIntakeRollersKs;  // Static friction FF
        slot0.kV = IntakeRollerConstants.kIntakeRollersKv;  // Velocity FF
        slot0.kA = 0;  // No accel FF for velocity

        intakeRollerMotor.getConfigurator().apply(configs);
    }

    /**
     * Sets intake velocity in rotations per second (RPS). Positive for intake, negative for outtake.
     * @param velocityRPS Target velocity (e.g., 10 RPS)
     */
    public void setVelocity(double velocityRPS) {
        velocityReq.Velocity = velocityRPS;
        intakeRollerMotor.setControl(velocityReq);
    }

    /** Stops the motor with neutral output */
    public void stopMotor() {
        intakeRollerMotor.setControl(neutralOut);
    }

    /** Legacy voltage control (use sparingly for open-loop testing) */
    public void setMotorPower(double percentOutput) {
        intakeRollerMotor.set(percentOutput);
    }

    // Getters for debugging/tuning
    public TalonFX getIntakeRollerMotor() {
        return intakeRollerMotor;
    }

    public double getVelocityRPS() {
        return intakeRollerMotor.getVelocity().getValueAsDouble();
    }

    public boolean isIntakeRollersRunning() {
        return Math.abs(getVelocityRPS()) > 0.5;  // Threshold for "running"
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IntakeRollers Running", isIntakeRollersRunning());
        SmartDashboard.putNumber("Intake Velocity RPS", getVelocityRPS());
    }
}