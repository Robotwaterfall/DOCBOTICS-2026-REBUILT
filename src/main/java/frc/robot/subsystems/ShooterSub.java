package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
    /*
     * /* Shooter controllers to shoot fuel
     */
    public TalonFX shooter_lead = new TalonFX(ShooterConstants.kShooterLeadMotorPort);
    public TalonFX shooterFollower_1 = new TalonFX(ShooterConstants.kShooterFollower_1_port);
    public TalonFX shooterFollower_2 = new TalonFX(ShooterConstants.kShooterFollower_2_port);
    public TalonFX indexMotor = new TalonFX(ShooterConstants.kIndexer_port);



    public ShooterSub() {

    }

    // *TODO figure out direction of motors relative to direction of current. */
    public void setShooterMotorsPower(double shootPower, double indexPower) {
        /*
         * power shooter lead motor, both follower motors and index will follow
      
         */
        shooter_lead.set(shootPower);
        shooterFollower_1.set(shootPower);
        shooterFollower_2.set(shootPower);
        indexMotor.set(indexPower);

    }

    public void StopMotors() {
        /* Stop shooter motors and indexer. */
        shooter_lead.set(0.0);
        shooterFollower_1.set(0.0);
        shooterFollower_2.set(0.0);
        indexMotor.set(0);

        shooter_lead.stopMotor();
        shooterFollower_1.stopMotor();
        shooterFollower_2.stopMotor();
        indexMotor.stopMotor();

    }
}