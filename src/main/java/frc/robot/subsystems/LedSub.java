package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LedSub extends SubsystemBase {
  private final Spark Blinkin;
  private final Spark Blinkin2;
  private Timer timer = new Timer();
  private boolean isOn = false;

  public LedSub() {
    Blinkin = new Spark(LEDConstants.BLINKIN_PWM_PORT);
    Blinkin2 = new Spark(LEDConstants.BLINKIN2_PWM_PORT);
    timer.start();
  }

  public void setRawPattern(double value) {
    Blinkin.set(value);
    Blinkin2.set(value);
  }

  public void setOrangeBlink() {
    if (timer.get() >= 0.2) {
      isOn = !isOn;
      timer.reset();

      if (isOn) {
        Blinkin.set(LEDConstants.BLINKIN_PATTERN_ORANGE_BLINK_POS); 
        Blinkin2.set(LEDConstants.BLINKIN_PATTERN_ORANGE_BLINK_POS); 
      } else {
        Blinkin.set(0.99);
        Blinkin2.set(0.99);
      }
    }
  }

  public void disable() {
    Blinkin.stopMotor();
    Blinkin2.stopMotor();
  }
}