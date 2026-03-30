package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simple subsystem to control a REV Blinkin (PWM) or other PWM-based LED controller.
 *
 * Note: The numeric "raw" values used for patterns are placeholders. Replace them with the
 * correct values from REV Blinkin documentation for the solid red, orange blink and RGB/rainbow
 * patterns you want.
 */
public class LedSub extends SubsystemBase {
  private final Servo pwm;

  public LedSub() {
    pwm = new Servo(Constants.LEDConstants.BLINKIN_PWM_PORT);
  }

  /** Set the raw PWM value (device-dependent). */
  public void setRawPattern(double position) {
    // Use Servo.setPosition-style normalized values (0..1). The constants in
    // `Constants.LEDConstants` are placeholders and should be tuned to map to
    // the actual Blinkin patterns you want.
    pwm.setPosition(position);
  }

  public void setSolidRed() {
    setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_SOLID_RED_POS);
  }

  public void setOrangeBlink() {
    setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_ORANGE_BLINK_POS);
  }

  public void setRainbow() {
    setRawPattern(Constants.LEDConstants.BLINKIN_PATTERN_RAINBOW_POS);
  }

  /** Convenience: disable PWM output (not generally needed for Blinkin). */
  public void disable() {
    pwm.setPosition(0.0);
  }
}
