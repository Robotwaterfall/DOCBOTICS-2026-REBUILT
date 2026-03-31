package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSub;

public class StopShooterMotorsCMD extends Command{
    ShooterSub shooterSub;

    public StopShooterMotorsCMD(ShooterSub shooterSub){
        // No subsystem requirements since this command is meant to be used as an interrupting command
        this.shooterSub = shooterSub;
        addRequirements(shooterSub);
    }

    @Override
    public boolean isFinished() {
        return true; // This command finishes immediately after executing
    }

    @Override
    public void end(boolean interrupted) {
        shooterSub.stopShooterMotors(); // Stop the shooter motors by setting velocity to 0
    }

}