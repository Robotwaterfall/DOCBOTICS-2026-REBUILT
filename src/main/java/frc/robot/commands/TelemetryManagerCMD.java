package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSub;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.PoseManager;

public class TelemetryManagerCMD extends Command{
    private final SwerveSub swerveSub;

    public TelemetryManagerCMD(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;
        // No addRequirements() needed for passive pose reading
    }

    @Override
    public void execute() {
        Pose2d pose = swerveSub.getPose();
        SmartDashboard.putNumber("Pose X (m)", pose != null ? pose.getX() : 0.0);
        SmartDashboard.putNumber("Pose Y (m)", pose != null ? pose.getY() : 0.0);
        SmartDashboard.putNumber("Pose Heading (deg)", pose != null ? pose.getRotation().getDegrees() : 0.0);

        Pose2d hubPose = PoseManager.getAllianceHubPose2d();
        SmartDashboard.putNumber("Hub X (m)", hubPose.getX());
        SmartDashboard.putNumber("Hub Y (m)", hubPose.getY());

        SmartDashboard.putNumber("DistanceToHub (in)", PoseManager.getDistanceToHubInches(swerveSub));
        SmartDashboard.putNumber("HeadingErrorToHub (deg)", PoseManager.getHeadingErrorDegreesHub(swerveSub));
        SmartDashboard.putBoolean("InAllianceZone", PoseManager.isInAllianceZone(swerveSub));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
