package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {

    public TalonFX shooterLead = new TalonFX(ShooterConstants.kShooterLeadMotorId);
    public TalonFX shooterFollowerRight = new TalonFX(ShooterConstants.kShooterFollowerRightId);
    // public TalonFX shooterFollowerLeft = new TalonFX(ShooterConstants.kShooterFollowerLeftId);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static double desiredVelocityFPS;

    private double tunableVelocityFps = 0.0;  // Start at 0, tune via buttons

    public ShooterSub() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kS = ShooterConstants.kShooterKs;
        cfg.Slot0.kV = ShooterConstants.kShooterKv;
        cfg.Slot0.kP = ShooterConstants.kShooterKP;
        cfg.Slot0.kI = ShooterConstants.kShooterKi;
        cfg.Slot0.kD = ShooterConstants.kShooterKd;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterLead.getConfigurator().apply(cfg);
        shooterFollowerRight.getConfigurator().apply(cfg);
        // shooterFollowerLeft.getConfigurator().apply(cfg);
    }

    public void setShooterVelocityFPS(double wheelVelocityFeetPerSecond) {
        
        desiredVelocityFPS = wheelVelocityFeetPerSecond;

        double wheelVelocityInchesPerSecond = wheelVelocityFeetPerSecond * 12.0;

        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterInches;
        double wheelRps = wheelVelocityInchesPerSecond / wheelCircumference;

        double motorRps = wheelRps * ShooterConstants.kGearRatio;
        motorRps = Math.abs(motorRps);

        shooterLead.setControl(velocityRequest.withVelocity(motorRps));
        shooterFollowerRight.setControl(velocityRequest.withVelocity(motorRps));
        // shooterFollowerLeft.setControl(velocityRequest.withVelocity(motorRps));
    }

    public double getShooterLeadVelocityRPS() {
        return shooterLead.getVelocity().getValueAsDouble();
    }

    public double getShooterFollowerRightVelocityRPS() {
        return shooterFollowerRight.getVelocity().getValueAsDouble();
    }

    // public double getShooterFollowerLeftVelocityRPS() {
    //     return shooterFollowerLeft.getVelocity().getValueAsDouble();
    // }

    public double getShooterLeadVelocityFPS() {
        double motorRps = shooterLead.getVelocity().getValueAsDouble();
        double wheelRps = motorRps / ShooterConstants.kGearRatio;
        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterInches;
        double inchesPerSecond = wheelRps * wheelCircumference;
        return inchesPerSecond / 12.0;
    }

    public double getShooterFollowerRightVelocityFPS() {
        double motorRps = shooterFollowerRight.getVelocity().getValueAsDouble();
        double wheelRps = motorRps / ShooterConstants.kGearRatio;
        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterInches;
        double inchesPerSecond = wheelRps * wheelCircumference;
        return inchesPerSecond / 12.0;
    }

    // public double getShooterFollowerLeftVelocityFPS() {
    //     double motorRps = shooterFollowerLeft.getVelocity().getValueAsDouble();
    //     double wheelRps = motorRps / ShooterConstants.kGearRatio;
    //     double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterInches;
    //     double inchesPerSecond = wheelRps * wheelCircumference;
    //     return inchesPerSecond / 12.0;
    // }

    public double getDesiredVelocityFPS() {
        return desiredVelocityFPS;
    }

    // public double getAverageMotorVelocityFPS() {
    //     return (getShooterLeadVelocityFPS()
    //             + getShooterFollowerRightVelocityFPS()
    //             + getShooterFollowerLeftVelocityFPS()) / 3.0;
    // }

    public boolean isAtSetVelocityFPS() {
        return (getShooterLeadVelocityFPS()) > (desiredVelocityFPS - ShooterConstants.shooterTolerance)
                && (getShooterLeadVelocityFPS()) < (desiredVelocityFPS + ShooterConstants.shooterTolerance);
    }

    public void stopShooterMotors() {
        shooterLead.stopMotor();
        shooterFollowerRight.stopMotor();
        // shooterFollowerLeft.stopMotor();
    }

        // Expose getter/setter for tuning commands
    public void incrementVelocity(double deltaFps) {
        tunableVelocityFps += deltaFps;
        setShooterVelocityFPS(tunableVelocityFps);  // Apply immediately
    }

    public void decrementVelocity(double deltaFps) {
        tunableVelocityFps -= deltaFps;
        if (tunableVelocityFps < 0) tunableVelocityFps = 0;
        setShooterVelocityFPS(tunableVelocityFps);
    }

    public double getTunableVelocityFps() { return tunableVelocityFps; }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("currentDesiredVelocityFPS", desiredVelocityFPS);
        SmartDashboard.putNumber("shooterLeadVelocityFPS", getShooterLeadVelocityFPS());
        SmartDashboard.putNumber("shooterFollowerRightVelocityFPS", getShooterFollowerRightVelocityFPS());
        SmartDashboard.putBoolean("atDesiredVelocityFPS", isAtSetVelocityFPS());
    }

}