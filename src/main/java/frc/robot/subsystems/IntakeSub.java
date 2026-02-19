package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(0, MotorType.kBrushless);

    public void setMotorPower(double power) {
        intakeMotor.set(power);
    }

    public SparkMax getIntakeMotor() {
        return intakeMotor;
    }

  

}
