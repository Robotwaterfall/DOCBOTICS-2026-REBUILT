// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ResetHeadingCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.hoodServoAdjustCmd;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.SwerveSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveSub swerveSub = new SwerveSub();
  public final HoodSub hoodSub = new HoodSub();

  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

     // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kSlowModeIdx),
             /// By default will be on field oriented.
            () -> !
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
            () -> driverJoyStick.getRawButton(OIConstants.kLockWheelsButton))); 

    hoodSub.setDefaultCommand(
      new hoodServoAdjustCmd(hoodSub) //Hood should constantly be adjusting
    );
    
  }


  private void configureBindings() {

    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCmd(swerveSub)
    );
    
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
