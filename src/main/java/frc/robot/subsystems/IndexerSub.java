package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class IndexerSub extends SubsystemBase {

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

    // toString
    @Override
    public String toString() {
        String str = "";

        str += "Indexer Motor Power: " + this.indexerMotor.get();

        return str;
    }
}
