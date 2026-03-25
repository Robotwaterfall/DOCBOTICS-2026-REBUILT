package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstant;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ConveyorSub extends SubsystemBase {
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
}