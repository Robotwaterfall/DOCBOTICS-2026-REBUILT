package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.diagnostics.Diagnosable;
import frc.robot.diagnostics.DiagnosticResult;
import frc.robot.diagnostics.SystemCheck;

public class ShooterSub extends SubsystemBase implements Diagnosable, SystemCheck {

    public TalonFX shooterLead = new TalonFX(ShooterConstants.kShooterLeadMotorId);
    public TalonFX shooterFollowerRight = new TalonFX(ShooterConstants.kShooterFollowerRightId);
    public TalonFX shooterFollowerLeft = new TalonFX(ShooterConstants.kShooterFollowerLeftId);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static double desiredVelocityFPS;

    private double tunableVelocityFps = Constants.ShooterConstants.kWarmupVelocityFPS;  // Start at 0, tune via buttons

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
        shooterFollowerLeft.getConfigurator().apply(cfg);
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
        shooterFollowerLeft.setControl(velocityRequest.withVelocity(-motorRps));
    }

    public double getShooterLeadVelocityRPS() {
        return shooterLead.getVelocity().getValueAsDouble();
    }

    public double getShooterFollowerRightVelocityRPS() {
        return shooterFollowerRight.getVelocity().getValueAsDouble();
    }

    public double getShooterFollowerLeftVelocityRPS() {
        return shooterFollowerLeft.getVelocity().getValueAsDouble();
    }

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

    public double getShooterFollowerLeftVelocityFPS() {
        double motorRps = shooterFollowerLeft.getVelocity().getValueAsDouble();
        double wheelRps = motorRps / ShooterConstants.kGearRatio;
        double wheelCircumference = Math.PI * ShooterConstants.kWheelDiameterInches;
        double inchesPerSecond = wheelRps * wheelCircumference;
        return inchesPerSecond / 12.0;
    }

    public double getDesiredVelocityFPS() {
        return desiredVelocityFPS;
    }

    public double getAverageMotorVelocityFPS() {
        return (getShooterLeadVelocityFPS()
                + getShooterFollowerRightVelocityFPS()
                + getShooterFollowerLeftVelocityFPS()) / 3.0;
    }

    public boolean isAtSetVelocityFPS() {
        return (getShooterLeadVelocityFPS()) > (desiredVelocityFPS - ShooterConstants.shooterTolerance)
                && (getShooterLeadVelocityFPS()) < (desiredVelocityFPS + ShooterConstants.shooterTolerance);
    }

    public void stopShooterMotors() {
        shooterLead.stopMotor();
        shooterFollowerRight.stopMotor();
        shooterFollowerLeft.stopMotor();
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
        SmartDashboard.putNumber("shooterFollowerLeftVelocityFPS", getShooterFollowerLeftVelocityFPS());
        SmartDashboard.putNumber("shooterAvgVelocityFPS", getAverageMotorVelocityFPS());
        SmartDashboard.putNumber("shooterVelocityErrorFPS", desiredVelocityFPS - getAverageMotorVelocityFPS());
        SmartDashboard.putBoolean("atDesiredVelocityFPS", isAtSetVelocityFPS());
    }

    /**
     * Description: Checks the encoder connectivity and if the shooters respond to voltage
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: A DiagnosticResult object is returned with the results of the diagnostics
     * @return The result of the Diagnostic 
     */
    @Override
    public DiagnosticResult runDiagnostics() {
        DiagnosticResult result = new DiagnosticResult("Shooter");
        boolean encoderConnected;

        // Encoder conectivity check (1) (Single check since this won't change)
        // Lead
        encoderConnected = shooterLead.getPosition().getStatus().isOK();
        result.check("Lead encoder connected", encoderConnected);

        // Left
        encoderConnected = shooterFollowerLeft.getPosition().getStatus().isOK();
        result.check("Left encoder connected", encoderConnected);

        // Right
        encoderConnected = shooterFollowerRight.getPosition().getStatus().isOK();
        result.check("Right connected", encoderConnected);

        // Encoder change check (Matured check)
        // Lead
        result.checkRepeated(
            "Lead motor responds to voltage",
            () -> {
                double initial = shooterLead.getPosition().getValueAsDouble();

                shooterLead.set(0.2);
                Timer.delay(0.05);

                double newPos = shooterLead.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        shooterLead.set(0);

        // Left
        result.checkRepeated(
            "Left motor responds to voltage",
            () -> {
                double initial = shooterFollowerLeft.getPosition().getValueAsDouble();

                shooterLead.set(0.2);
                Timer.delay(0.05);

                double newPos = shooterLead.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        shooterFollowerLeft.set(0);

        // Left
        result.checkRepeated(
            "Right motor responds to voltage",
            () -> {
                double initial = shooterFollowerRight.getPosition().getValueAsDouble();

                shooterLead.set(0.2);
                Timer.delay(0.05);

                double newPos = shooterLead.getPosition().getValueAsDouble();
                return Math.abs(newPos - initial) > 0.01;
            },
            10,
            0.8
        );

        // Stop motor
        shooterFollowerRight.set(0);

        return result;
    }

    /**
     * Description: Performs a systems check of the shooter by ensuring it reaches warm up velocity
     * Pre-Condition: All objects and hardware are declared and initialized
     * Post-Condition: Systems check is performed and the diagnostic result is returned
     * @return The DiagnosticResult of the systems check
     */
    @Override
    public DiagnosticResult performSystemCheck() {
        DiagnosticResult result = new DiagnosticResult("IntakePitcerSC");

        // Shooter
        result.checkRepeated(
            "Reaches warm up velocity", 
            () -> {
                setShooterVelocityFPS(ShooterConstants.kWarmupVelocityFPS);

                double initial = getShooterLeadVelocityFPS();
                Timer.delay(0.05);

                return Math.abs(initial - getShooterLeadVelocityFPS()) > 0;
            },
            () -> isAtSetVelocityFPS(),
            0.8
        );

        // Stop motors
        setShooterVelocityFPS(0);
        stopShooterMotors();

        return result;
    }
}