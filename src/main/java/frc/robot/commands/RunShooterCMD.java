package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;

public class RunShooterCMD extends Command {

    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;

    private double desiredVelocity; // fps

    // 2‑param constructor
    // → get distance to hub, then set desiredVelocity from lookup table
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub) {
        this.shooterSub = shooterSub;
        this.swerveSub  = swerveSub;

        double targetDistanceInches = PoseManager.getDistanceToHubInches(swerveSub);
        double distanceFeet = targetDistanceInches / 12.0;

        ShooterLookup.ShooterParams params = ShooterLookup.getInterpolated(distanceFeet);
        this.desiredVelocity = params.velocityFps;

        addRequirements(shooterSub);
    }

    // 3‑param constructor
    // → desiredVelocity is passed explicitly
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub, double desiredVelocity) {
        this.shooterSub    = shooterSub;
        this.swerveSub     = swerveSub;
        this.desiredVelocity = desiredVelocity;

        addRequirements(shooterSub);
    }

    @Override
    public void execute() {
        shooterSub.setShooterVelocityFPS(desiredVelocity);

        SmartDashboard.putNumber("DesiredVelocity", desiredVelocity);
    }

    @Override
    public boolean isFinished() {
        // return shooterSub.isAtSetVelocityFPS();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stopShooterMotors();
    }
}
