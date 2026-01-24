package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
    /*
     * Note: In REV Hardware, the m_rightShooterFollow
     * controller must be configured to follow the m_leftShooterLead controller
     * and be inverted relative to the m_leftShooterLead controller’s direction.*
     * 
     * /* Shoter controllers to shooter fuel
     */
    public TalonFX shooter_lead = new TalonFX(ShooterConstants.kShooterLeadMotorPort);
    public TalonFX shooterFollower_1 = new TalonFX(ShooterConstants.kShooterFollower_1_port);
    public TalonFX shooterFollower_2 = new TalonFX(ShooterConstants.kShooterFollower_2_port);



    public ShooterSub() {

    }

    // *TODO figure out direction of motors relative to direction of current. */
    public void setShooterMotorsPower(double shootpower, double feederpower) {
        /*
         * power left lead motor, right motor will
         * follow if set correctly in REV hardware correctly
         */
        shooter_lead.set(shootpower);
        shooterFollower_1.set(shootpower);
        shooterFollower_2.set(shootpower);

    }

    public void StopMotors() {
        /* Stop shooter motors. */
        shooter_lead.set(0.0);
        shooterFollower_1.set(0.0);
        shooterFollower_2.set(0.0);
       

        shooter_lead.stopMotor();
        shooterFollower_1.stopMotor();
        shooterFollower_2.stopMotor();

    }
}