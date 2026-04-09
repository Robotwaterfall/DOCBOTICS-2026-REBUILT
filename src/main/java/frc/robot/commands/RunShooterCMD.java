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

        // May get better performance by only doing the velocity lookup once, but will require driver to re-trigger command if they move significantly.
        // Doing it in execute allows for real-time velocity updates as robot moves, but may cause performance issues.

        // if (distanceBased && PoseManager.isInAllianceZone(swerveSub)) {
        //     double distanceFeet = PoseManager.getDistanceToHubFeet(swerveSub);
        //     this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        // } else if (
        //         distanceBased && 
        //         PoseManager.isInWasteLand(swerveSub) && 
        //         PoseManager.isOnLeftSideOfField(swerveSub)){ // OG -> &&!PoseManager.isInAllianceZone(swerveSub) <- Redundant, if you are in neutral zone, you aren't in alliance zone
            
        //     double distanceFeet = PoseManager.getDistanceToLeftAllianceZoneMidpoint(swerveSub);
        //     this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        // } else if (
        //         distanceBased && 
        //         PoseManager.isInWasteLand(swerveSub) && 
        //         !PoseManager.isOnLeftSideOfField(swerveSub) && 
        //         !PoseManager.isInAllianceZone(swerveSub)){
            
        //     double distanceFeet = PoseManager.getDistanceToRightAllianceZoneMidpoint(swerveSub);
        //     this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        // } else {
        //     double distanceFeet = PoseManager.getDistanceToHubFeet(swerveSub);
        //     this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        // }
    }

    @Override
    public void execute() {

        // Computationally slow!!!! Consider guard clauses or something to reduce number of calls to PoseManager and ShooterLookup. 
        // Moved to execute to update shooter velocity in real time as robot moves, but may cause performance issues.
        // Consider caching distance/velocity and only updating every 0.5s or so.

        if (distanceBased && PoseManager.isInAllianceZone(swerveSub)) {
            double distanceFeet = PoseManager.getDistanceToHubFeet(swerveSub);
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        } else if (
                distanceBased && 
                PoseManager.isInWasteLand(swerveSub) && 
                PoseManager.isOnLeftSideOfField(swerveSub)){ // OG -> &&!PoseManager.isInAllianceZone(swerveSub) <- Redundant, if you are in neutral zone, you aren't in alliance zone
            
            double distanceFeet = PoseManager.getDistanceToLeftAllianceZoneMidpoint(swerveSub);
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        } else if (
                distanceBased && 
                PoseManager.isInWasteLand(swerveSub) && 
                !PoseManager.isOnLeftSideOfField(swerveSub)){ // see line 82 comment
            
            double distanceFeet = PoseManager.getDistanceToRightAllianceZoneMidpoint(swerveSub);
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        } else {
            double distanceFeet = PoseManager.getDistanceToHubFeet(swerveSub);
            this.desiredVelocity = ShooterLookup.getInterpolatedVelocity(distanceFeet);
        }

        shooterSub.setShooterVelocityFPS(desiredVelocity);
    }

}
