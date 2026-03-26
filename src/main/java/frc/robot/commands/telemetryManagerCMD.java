package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

public class telemetryManagerCMD extends Command{

    SwerveSub swerveSub;

    public telemetryManagerCMD() {
        

    }

    @Override
    public void execute() {
        //POSEMANAGER TELEMETRY
        SmartDashboard.putNumber("Pose: ", 0);
        SmartDashboard.putNumber("DistanceToHub (in): ", 0);
        SmartDashboard.putNumber("HeadingErrorToHub (deg): ", 0);
        SmartDashboard.putBoolean("InAllianceZone: ", PoseManager.isInAllianceZone(swerveSub));

        

        

    }

}
