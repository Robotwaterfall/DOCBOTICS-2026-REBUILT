package frc.robot.commands.pathfindingcommands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.PathPlannerConstants;

public class PathFindToPath extends Command {
    private final Command pathfindCommand;

    public PathFindToPath(String pathName) {
        PathConstraints constraints = new PathConstraints(
            PathPlannerConstants.kMaxPathfindingVel,
            PathPlannerConstants.kMaxPathfindingAccel,
            PathPlannerConstants.kMaxPathfindingAngVel,
            PathPlannerConstants.kMaxPathfindingAngAccel
        );

        Command tempCommand;

        try {
            PathPlannerPath targetPath = PathPlannerPath.fromPathFile(pathName);
            tempCommand = AutoBuilder.pathfindThenFollowPath(targetPath, constraints);
        } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError("Failed to load path '" + pathName + "': " + e.getMessage(), e.getStackTrace());
            tempCommand = Commands.none();
        }

        pathfindCommand = tempCommand;
    }

    @Override
    public void initialize() {
        pathfindCommand.initialize();
    }

    @Override
    public void execute() {
        pathfindCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathfindCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathfindCommand.isFinished();
    }
}