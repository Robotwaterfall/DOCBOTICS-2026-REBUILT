package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

/**
 * Tuning command: hold to keep shooter spinning.
 * D-pad LEFT/RIGHT adjusts velocity live.
 * Release to stop.
 */
public class ShooterTuningCMD extends Command {

    private final ShooterSub shooterSub;
    private final PS5Controller controller;

    private double velocityFps;

    private boolean prevRight = false;
    private boolean prevLeft = false;

    private static final double VELOCITY_STEP = 5.0;

    public ShooterTuningCMD(ShooterSub shooterSub, PS5Controller controller) {
        this.shooterSub = shooterSub;
        this.controller = controller;

        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
        velocityFps = 0;
        prevRight = false;
        prevLeft = false;
    }

    @Override
    public void execute() {
        int pov = controller.getPOV();
        boolean right = (pov == 90);
        boolean left = (pov == 270);

        if (right && !prevRight) {
            velocityFps += VELOCITY_STEP;
        }
        if (left && !prevLeft) {
            velocityFps -= VELOCITY_STEP;
            if (velocityFps < 0) velocityFps = 0;
        }

        prevRight = right;
        prevLeft = left;

        shooterSub.setShooterVelocityFPS(velocityFps);

        SmartDashboard.putNumber("Tuning_VelocityFPS", velocityFps);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stopShooterMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}