package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LedSub;
import frc.robot.util.HubLogic;



public class PeriodicLightsCMD extends Command {
    private final LedSub ledSub;
    private final HubLogic hubLogic = new HubLogic();

    public PeriodicLightsCMD(LedSub ledSub) {
        this.ledSub = ledSub;
        addRequirements(ledSub);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      Optional<Alliance> ally = DriverStation.getAlliance();

    if (DriverStation.isDisabled()) {
      ledSub.disable();
    } else if (DriverStation.isAutonomous()) {
    if (DriverStation.isFMSAttached()) {
      ledSub.setRawPattern(LEDConstants.BLINKIN_PATTERN_RAINBOW_POS);
    } else {
      ledSub.setOrangeBlink();
    }
    } else if (DriverStation.isTeleop()) {
      if (DriverStation.isFMSAttached()) {

switch (hubLogic.getHubState()) {
  case ACTIVE:
    ledSub.setRawPattern(LEDConstants.BLINKIN_PATTERN_RAINBOW_POS);
    break;
  case INACTIVE:
          if (ally.get() == Alliance.Red) {
            ledSub.setRawPattern(LEDConstants.BLINKIN_PATTERN_BLUE_POS);
          } else {
            ledSub.setRawPattern(LEDConstants.BLINKIN_PATTERN_RED_POS);
          }
    break;
  case WARNING:
    ledSub.setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_SHIFT);
    break;
}
      } else {
        ledSub.setOrangeBlink();
      }
    }
    }
    
    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return false;
    }
}