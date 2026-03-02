// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.autoConstants;
import frc.robot.commands.ResetHeadingCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TeleOpIntakePowerCmd;
import frc.robot.commands.adaptableShooterCmd;
import frc.robot.commands.runConveyorCmd;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.AutoIntakePowerCmd;
import frc.robot.commands.HoodServoAdjustCmd;
import frc.robot.commands.IdleIntakePitcherCmd;
import frc.robot.commands.MoveIntakePitcherCmd;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IntakePitcherSub;
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
  public final HoodSub hoodSub = new HoodSub();
  public final ShooterSub shooterSub = new ShooterSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ConveyorSub conveyorSub = new ConveyorSub();

  private final XboxController driverJoyStick = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;


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

    
    intakeSub.setDefaultCommand(new TeleOpIntakePowerCmd(intakeSub, 
      () -> driverJoyStick.getRightBumperButtonPressed(), 
      () -> driverJoyStick.getLeftBumperButtonPressed())); 
    
    

    hoodSub.setDefaultCommand(
      new HoodServoAdjustCmd(hoodSub) //Hood should constantly be adjusting
    );

    shooterSub.setDefaultCommand(
      new adaptableShooterCmd(shooterSub, hoodSub, 
      () -> driverJoyStick.getRightTriggerAxis(),
      false)
    );

    intakePitcherSub.setDefaultCommand(new IdleIntakePitcherCmd(intakePitcherSub));

    autoChooser = AutoBuilder.buildAutoChooser(); //Default auto will be 'Commands.none()'
    SmartDashboard.putData("AutoMode: ", autoChooser);
    
  }


  private void configureBindings() {

    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCmd(swerveSub)
    );

    //This command runs the shooter routine
    Command adaptableShootCommand = 
      new ParallelDeadlineGroup(
        
      new WaitCommand(autoConstants.timeElapsedShootingSecounds),
        
      new ParallelCommandGroup(

          new adaptableShooterCmd(shooterSub, hoodSub,() -> 1.0, true),
          new HoodServoAdjustCmd(hoodSub),

          new SequentialCommandGroup(
            new MoveIntakePitcherCmd(intakePitcherSub, IntakePitcherConstants.kPitcherIn),
            new WaitCommand(autoConstants.timeBetweenPitcherInAndOut),
            new MoveIntakePitcherCmd(intakePitcherSub, IntakePitcherConstants.kPitcherOut)
          ).repeatedly()

        ));
    NamedCommands.registerCommand("adaptableShooterCommand", adaptableShootCommand);

    Command intakeFor5Secounds = 
      new SequentialCommandGroup(
        new AutoIntakePowerCmd(intakeSub, 5)
      );
    NamedCommands.registerCommand("intakeFor5Secounds", intakeFor5Secounds);



  
  
    
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
