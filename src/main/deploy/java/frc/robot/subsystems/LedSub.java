package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LedSub extends SubsystemBase {
  private final Spark Blikin;
  private Timer timer = new Timer();
  private boolean isOn = false;

  public LedSub() {
    Blikin = new Spark(Constants.LEDConstants.BLINKIN_PWM_PORT);
    timer.start();
  }

  public void setRawPattern(double value) {
    Blikin.set(value);
  }

  public void setSolidRed() {
    setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_SOLID_RED_POS);
  }

  public void setOrangeBlink() {
    if (timer.get() >= 0.2) {
      isOn = !isOn;
      timer.reset();

      if (isOn) {
        Blikin.set(Constants.LEDConstants.BLINKIN_PATTERN_ORANGE_BLINK_POS); 
      } else {
        Blikin.set(0.99);
      }
    }
  }

  public void setRainbow() {
    setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_RAINBOW_POS);
  }

  public void disable() {
    //Most likely wont need to be called tbh.
    Blikin.set(0);
  }
}