package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;

public class LockWheelsCMD extends Command {
    private final SwerveSub swerveSub;

    private final SwerveModuleState[] desiredLockOnStates = new SwerveModuleState[] {
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), // front right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),  // front left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),  // back right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45))  // back left
    };

    public LockWheelsCMD(SwerveSub swerveSub) {
        this.swerveSub = swerveSub;
        addRequirements(swerveSub);
    }

    @Override
    public void initialize() {
        swerveSub.stopModules();
    }

    @Override
    public void execute() {
        swerveSub.setModuleStates(desiredLockOnStates, false);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}