package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSub;



public class LightsRslCMD extends Command {
    private final LedSub ledSub;

    public LightsRslCMD(LedSub ledSub) {
        this.ledSub = ledSub;
        addRequirements(ledSub);
    }
    
    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        ledSub.setOrangeBlink();
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
