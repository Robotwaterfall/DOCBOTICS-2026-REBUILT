// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.PitchIntakeCMD;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveSub swerveSub = new SwerveSub();
  public final IntakeSub intakeSub = new IntakeSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ShooterSub shooterSub = new ShooterSub();

  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);


  private final JoystickButton intakeFueldButton = new JoystickButton(driverJoyStick,OIConstants.kIntakeFuelButton);
  private final JoystickButton powerConveyorButton = new JoystickButton(driverJoyStick,OIConstants.kPowerConveryorButton);
  private final JoystickButton pitchIntakeUpButton = new JoystickButton(driverJoyStick,OIConstants.kPitchIntakeUp);
  private final JoystickButton pitchIntakeDownButton = new JoystickButton(driverJoyStick,OIConstants.kPitchIntakeDown);
  private final JoystickButton shootFuelButton = new JoystickButton(driverJoyStick,OIConstants.kPitchIntakeDown);

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

    intakePitcherSub.setDefaultCommand(new PitchIntakeCMD(intakePitcherSub, true));

    

  }


  private void configureBindings() {

    
    
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
