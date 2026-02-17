
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSub extends SubsystemBase {


    private SparkMax intakeConsumerMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    

    public SparkMax getIntakeConsumerMotor() {
        return intakeConsumerMotor;
    }





}