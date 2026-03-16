package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyorSub extends SubsystemBase {
    private SparkMax conveyorMotor = new SparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);

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
        return conveyorPower > 0 || conveyorPower < 0;
    }

    @Override
    public String toString(){
        String str = "";
        str += "isConveyorOn: " + isConveyorOn(); //To check if conveyor is on
        return str;
    }
}