package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    public TalonFX indexMotor = new TalonFX(ShooterConstants.kIndexMotor_Port);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static double desiredVelocity;



    public ShooterSub() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Example gains, you must tune these
        cfg.Slot0.kP = Constants.ShooterConstants.kShooterKP;
        cfg.Slot0.kI = Constants.ShooterConstants.kShooterKi;
        cfg.Slot0.kD = Constants.ShooterConstants.kShooterKd;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooter_lead.getConfigurator().apply(cfg);

    }

   

    public void setShooterVelocityMPS(double wheelVelocityMetersPerSecond){

         // wheel m/s -> wheel rotations per second
        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterMeters;
        double wheelRps = wheelVelocityMetersPerSecond / wheelCircumference;

        // wheel rps -> motor rps (motorRot / wheelRot)
        double motorRps = wheelRps * ShooterConstants.kGearRatio;

        shooter_lead.setControl(velocityRequest.withVelocity(motorRps));
        shooterFollower_1.setControl(velocityRequest.withVelocity(motorRps));
        shooterFollower_2.setControl(velocityRequest.withVelocity(motorRps));

    }

    public double getShooter_LeadVelocity(){
        return shooter_lead.getVelocity().getValueAsDouble();
    }

    public double getShooterFollower_1Velocity(){
        return shooter_lead.getVelocity().getValueAsDouble();
    }

    public double getShooterFollower_2Velocity(){
        return shooter_lead.getVelocity().getValueAsDouble();
    }

    public double getDesiredVelocity(){
        return desiredVelocity;
    }

    public void setDesiredVelocity(double desiredV){
        desiredVelocity = desiredV;
    }

    public void setIndexSpeed(double indexPower){
        indexMotor.set(indexPower);
    }

   

    public void stopMotors() {
        /* Stop shooter motors. */
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