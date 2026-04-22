package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSub;


public class SetLightsCMD extends Command {
    private final LedSub ledSub;
    private final double lightValue;

    public SetLightsCMD(LedSub ledSub, double lightValue) {
        this.ledSub = ledSub;
        this.lightValue = lightValue;
        addRequirements(ledSub);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ledSub.setRawPattern(lightValue);
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}