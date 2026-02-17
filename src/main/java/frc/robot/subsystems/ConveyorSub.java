
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyorSub extends SubsystemBase {

    /**Motor that allows the intake to consume coral. */
    private SparkMax conveyorMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    
    /**
     * @return the intake motor that consumes the Coral.
     */
    public SparkMax getConveryorMotor() {
        return conveyorMotor;
    }





}