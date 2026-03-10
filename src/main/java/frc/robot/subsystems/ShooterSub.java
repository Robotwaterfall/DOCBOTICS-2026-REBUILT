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
     * Note: In phoenix tuner Hardware, the m_rightShooterFollow
     * controller must be configured to follow the m_leftShooterLead controller
     * and be inverted relative to the m_leftShooterLead controller’s direction.*
     * 
     * /* Shoter controllers to shooter fuel
     */
    public TalonFX shooterLead = new TalonFX(ShooterConstants.kShooterLeadMotorId);
    public TalonFX shooterFollowerRight = new TalonFX(ShooterConstants.kShooterFollowerRightId);
    public TalonFX shooterFollowerLeft = new TalonFX(ShooterConstants.kShooterFollowerLeftId);

    public TalonFX indexMotor = new TalonFX(ShooterConstants.kIndexMotorId);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static double desiredVelocity;

    public ShooterSub() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kS = Constants.ShooterConstants.kShooterKs;
        cfg.Slot0.kV = Constants.ShooterConstants.kShooterKv;
        cfg.Slot0.kP = Constants.ShooterConstants.kShooterKP;
        cfg.Slot0.kI = Constants.ShooterConstants.kShooterKi;
        cfg.Slot0.kD = Constants.ShooterConstants.kShooterKd;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterLead.getConfigurator().apply(cfg);
        shooterFollowerRight.getConfigurator().apply(cfg);
        shooterFollowerLeft.getConfigurator().apply(cfg);

    }

   

    public void setShooterVelocityMPS(double wheelVelocityMetersPerSecond){

         // wheel m/s -> wheel rotations per second
        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterMeters;
        double wheelRps = wheelVelocityMetersPerSecond / wheelCircumference;

        // wheel rps -> motor rps (motorRot / wheelRot)
        double motorRps = wheelRps * ShooterConstants.kGearRatio;

        shooterLead.setControl(velocityRequest.withVelocity(motorRps));
        shooterFollowerRight.setControl(velocityRequest.withVelocity(motorRps));
        shooterFollowerLeft.setControl(velocityRequest.withVelocity(motorRps));

        

    }

    public double getShooterLeadVelocity(){
        return shooterLead.getVelocity().getValueAsDouble();
    }

    public double getShooterFollowerRightVelocity(){
        return shooterFollowerRight.getVelocity().getValueAsDouble();
    }

    public double getShooterFollowerLeftVelocity(){
        return shooterFollowerLeft.getVelocity().getValueAsDouble();
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

    public double getAverageShootingVelocityMPS(){
        double averageVelocity = (shooterLead.getVelocity().getValueAsDouble() 
                                    + 
                                    shooterFollowerRight.getVelocity().getValueAsDouble()
                                    + 
                                    shooterFollowerLeft.getVelocity().getValueAsDouble()) 
                                    / 3;

        return averageVelocity;
    }

    public boolean isAtSetVelocityMPS(){
        return  (getShooterLeadVelocity()) > (desiredVelocity - ShooterConstants.shooterTolerance) &&
                (getShooterLeadVelocity()) < (desiredVelocity + ShooterConstants.shooterTolerance);
    }

    public void stopShooterMotors() {
        shooterLead.stopMotor();
        shooterFollowerRight.stopMotor();
        shooterFollowerLeft.stopMotor();
    }

    public void stopIndexMotor(){
        indexMotor.stopMotor();
    }

    @Override
    public String toString(){

        String str = " ";

        str += "Shooter Subsystem Information";
        str += "currentDesiredVelocityMPS: " + getDesiredVelocity();
        str += "averageVelocityMPS: " + getAverageShootingVelocityMPS();
        str += "atDesiredVelocityMPS: " + isAtSetVelocityMPS();

        return str;

        }

            
    }
