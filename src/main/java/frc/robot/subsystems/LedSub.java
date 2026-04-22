package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LedSub extends SubsystemBase {
  private final Spark Blikin;
  private Timer timer = new Timer();
  private boolean isOn = false;

  public LedSub() {
    Blikin = new Spark(LEDConstants.BLINKIN_PWM_PORT);
    timer.start();
  }

  public void setRawPattern(double value) {
    Blikin.set(value);
  }

  public void setOrangeBlink() {
    if (timer.get() >= 0.2) {
      isOn = !isOn;
      timer.reset();

      if (isOn) {
        Blikin.set(LEDConstants.BLINKIN_PATTERN_ORANGE_BLINK_POS); 
      } else {
        Blikin.set(0.99);
      }
    }
  }

  public void disable() {
    Blikin.stopMotor();
  }
}