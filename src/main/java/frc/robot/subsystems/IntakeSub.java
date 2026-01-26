
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeSub extends SubsystemBase {

    /**Motor that allows the intake to consume fuel. */
    private SparkMax intakeConsumerMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    
    /**
     * @return the intake motor that consumes the fuel.
     */
    public SparkMax getIntakeConsumerMotor() {
        return intakeConsumerMotor;
    }





}