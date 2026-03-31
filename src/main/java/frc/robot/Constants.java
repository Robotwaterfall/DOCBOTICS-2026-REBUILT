// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class LEDConstants {
    // PWM port on the RoboRIO where the Blinkin is connected (change as needed).
    public static final int BLINKIN_PWM_PORT = 0;

    // Raw PWM values for patterns. These are placeholders — please replace with the values
    // from the Blinkin documentation or your working mapping.
    // Patterns expressed as normalized position values (0.0 - 1.0) for use with Servo.setPosition()
    // These are placeholders — replace with values from the Blinkin documentation / your mapping.
    public static final double BLINKIN_PATTERN_RED_POS = -0.25;
    public static final double BLINKIN_PATTERN_BLUE_POS = -0.23;
    public static final double BLINKIN_PATTERN_ORANGE_BLINK_POS = 0.65;
    public static final double BLINKIN_PATTERN_RAINBOW_POS = -0.99; 
    public static final double BLINKIN_PATTERN_DOC_POS = -0.99; 
  }
}
