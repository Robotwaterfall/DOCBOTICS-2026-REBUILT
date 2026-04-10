package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.diagnostics.Diagnosable;
import frc.robot.diagnostics.DiagnosticResult;
import frc.robot.diagnostics.SystemCheck;

public class IntakeRollersSub extends SubsystemBase implements Diagnosable, SystemCheck {
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

    /**
     * Description: Checks the encoder connectivity and if the rollers respond to voltage
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: A DiagnosticResult object is returned with the results of the diagnostics
     * @return The result of the Diagnostic 
     */
    @Override
    public DiagnosticResult runDiagnostics() {
        DiagnosticResult result = new DiagnosticResult("IntakeRollers");

        // Encoder conectivity check (1) (Single check since this won't change)
        boolean encoderConnected = intakeRollerMotor.getPosition().getStatus().isOK();
        result.check("Encoder connected", encoderConnected);

        // Encoder change check (2) (Matured check)
        result.checkRepeated(
            "Motor responds to voltage",
            () -> {
                double initial = intakeRollerMotor.getPosition().getValueAsDouble();

                intakeRollerMotor.set(0.2);
                Timer.delay(0.05);

                double newPos = intakeRollerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        intakeRollerMotor.set(0);

        return result;
    }

    /**
     * Description: Performs a systems check of the rollers by intaking and outtaking
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: Systems check is performed and the diagnostic result is returned
     * @return The DiagnosticResult of the systems check
     */
    @Override
    public DiagnosticResult performSystemCheck() {
        DiagnosticResult result = new DiagnosticResult("IntakeRollerSC");

        // Index Forward
        result.checkRepeated(
            "Intake", 
            () -> {
                double initial = intakeRollerMotor.getPosition().getValueAsDouble();

                setMotorPower(IntakeRollerConstants.kIntakePower);

                double newPos = intakeRollerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            }, 
            100,
            0.8
        );

        // Stop Motor
        setMotorPower(0);
        stopMotor();

        // Index Reverse
        result.checkRepeated(
            "Outake", 
            () -> {
                double initial = intakeRollerMotor.getPosition().getValueAsDouble();

                setMotorPower(IntakeRollerConstants.kOuttakePower);

                double newPos = intakeRollerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            }, 
            100,
            0.8        
        );

        // Stop Motor
        setMotorPower(0);
        stopMotor();

        return result;
    }
}
