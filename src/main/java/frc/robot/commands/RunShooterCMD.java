package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;
import frc.robot.util.ShooterLookup;

public class RunShooterCMD extends InstantCommand {

    private final ShooterSub shooterSub;
    private final SwerveSub swerveSub;

    private double desiredVelocity; // fps
    private final boolean distanceBased;

    // 1‑param constructor
    // → distance-based: looks up velocity from Limelight distance when command starts
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub) {
        this.shooterSub = shooterSub;
        this.swerveSub = swerveSub;
        this.desiredVelocity = 0;
        this.distanceBased = true;

        addRequirements(shooterSub);
    }

    // 2‑param constructor
    // → desiredVelocity is passed explicitly
    public RunShooterCMD(ShooterSub shooterSub, SwerveSub swerveSub, double desiredVelocity) {
        this.shooterSub = shooterSub;
        this.swerveSub = swerveSub;
        this.desiredVelocity = desiredVelocity;
        this.distanceBased = false;

        addRequirements(shooterSub);
    }

    @Override
    public void initialize() {
        if (distanceBased) {
            double distanceFeet = PoseManager.getDistanceToHubFeet(swerveSub);
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        }
    }

    @Override
    public void execute() {
        shooterSub.setShooterVelocityFPS(desiredVelocity);
    }

}
