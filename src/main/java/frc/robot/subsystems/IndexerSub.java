package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.diagnostics.Diagnosable;
import frc.robot.diagnostics.DiagnosticResult;
import frc.robot.diagnostics.SystemCheck;

public class IndexerSub extends SubsystemBase implements Diagnosable, SystemCheck {

    public TalonFX indexerMotor = new TalonFX(ShooterConstants.kIndexMotorId);


    /**
     * Description: Sets motor to desired power
     * Pre-Condition: All objects and hardware must be initialized and param must be a double between -1 and 1
     * Post-Condition: Intake motor power is set
     * @param indexPower
     */
    public void setIndexSpeed(double indexPower){
        indexPower = MathUtil.clamp(indexPower, -1, 1);
        
        indexerMotor.set(indexPower);
    }

    /**
     * Description: Stops the motor
     * Pre-Condition: Motor must be initalized
     * Post-Condition: Motor is stopped
     */
    public void stopIndexMotor(){
        indexerMotor.stopMotor();
    }

    public boolean isIndexMotorRunning() {
        return indexerMotor.get() != 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isIndexMotorRunning", isIndexMotorRunning());
    }

    /**
     * Description: Checks the encoder connectivity and if the indexer responds to voltage
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: A DiagnosticResult object is returned with the results of the diagnostics
     * @return The result of the Diagnostic 
     */
    @Override
    public DiagnosticResult runDiagnostics() {
        DiagnosticResult result = new DiagnosticResult("Indexer");

        // Encoder conectivity check (1) (Single check since this won't change)
        boolean encoderConnected = indexerMotor.getPosition().getStatus().isOK();
        result.check("Encoder connected", encoderConnected);

        // Encoder change check (2) (Matured check)
        result.checkRepeated(
            "Motor responds to voltage",
            () -> {
                double initial = indexerMotor.getPosition().getValueAsDouble();

                indexerMotor.set(0.2);
                Timer.delay(0.05);

                double newPos = indexerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        indexerMotor.set(0);

        return result;
    }

    /**
     * Description: Performs a systems check of the indexer by moving it forwards and backwards
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: Systems check is performed and the diagnostic result is returned
     * @return The DiagnosticResult of the systems check
     */
    @Override
    public DiagnosticResult performSystemCheck() {
        DiagnosticResult result = new DiagnosticResult("IndexerSC");

        // Index Forward
        result.checkRepeated(
            "Index Forward", 
            () -> {
                double initial = indexerMotor.getPosition().getValueAsDouble();

                setIndexSpeed(ShooterConstants.kIndexSpeed);
                Timer.delay(0.05);

                double newPos = indexerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            }, 
            100,
            0.8
        );

        // Stop Motor
        setIndexSpeed(0);
        stopIndexMotor();

        // Index Reverse
        result.checkRepeated(
            "Index Reverse", 
            () -> {
                double initial = indexerMotor.getPosition().getValueAsDouble();

                setIndexSpeed(ShooterConstants.kReverseIndexSpeed);
                Timer.delay(0.05);

                double newPos = indexerMotor.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            }, 
            100,
            0.8
        );

        // Stop Motor
        setIndexSpeed(0);
        stopIndexMotor();

        return result;
    }

}
