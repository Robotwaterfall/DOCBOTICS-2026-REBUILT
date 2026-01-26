
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ConveyorSub extends SubsystemBase {

    /**Motor powes the conveyor motor. */
    private SparkMax conveyorMotor = new SparkMax(ConveyorConstants.kConveyorMotorPort, MotorType.kBrushless);
    
    /**
     * @return the converyor motor moves the fuel up to shooter
     */
    public SparkMax getConveryorMotor() {
        return conveyorMotor;
    }





}