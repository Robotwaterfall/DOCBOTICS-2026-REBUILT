package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstant;
import frc.robot.diagnostics.Diagnosable;
import frc.robot.diagnostics.DiagnosticResult;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyorSub extends SubsystemBase implements Diagnosable {
    private SparkMax conveyorMotor = new SparkMax(ConveyorConstant.kConveyorMotorPort, MotorType.kBrushless);

    private double conveyorPower = 0;

    public SparkMax getConveyorMotor() {
        return conveyorMotor;
    }

    public void setConveyorPower(double power){
        conveyorMotor.set(power);
        conveyorPower = power;
    }

    public double getConveyorPower(){
        return conveyorPower;
    }

    public boolean isConveyorOn(){
        return conveyorMotor.get() != 0;
    }

    @Override
    public void periodic() {
       SmartDashboard.putBoolean("isConveyorOn", isConveyorOn());

    }

    /**
     * Description: Checks the encoder connectivity and if the conveyer responds to voltage
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: A DiagnosticResult object is returned with the results of the diagnostics
     * @return The result of the Diagnostic 
     */
    @Override
    public DiagnosticResult runDiagnostics() {
        DiagnosticResult result = new DiagnosticResult("Conveyor");

        // Encoder conectivity check (1) (Single check since this won't change)
        double initialPosition = conveyorMotor.getEncoder().getPosition();
        boolean encoderConnected = !Double.isNaN(initialPosition); // SparkMax encoders return NaN if disconnected
        result.check("Encoder connected", encoderConnected);

        // Encoder change check (2) (Matured check)    
        result.checkRepeated(
            "Motor responds to voltage",
            () -> {
                double initial = conveyorMotor.getEncoder().getPosition();

                conveyorMotor.set(0.2);
                Timer.delay(0.05);
                
                double newPos = conveyorMotor.getEncoder().getPosition();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        conveyorMotor.set(0);

        return result;
    }

}