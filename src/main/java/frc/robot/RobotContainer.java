// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTargetCMD;
import frc.robot.commands.DecrementShooterCMD;
import frc.robot.commands.IncrementShooterCMD;
import frc.robot.commands.LockWheelsCMD;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.commands.PeriodicLightsCMD;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.StopShooterMotorsCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.commandgroups.FireShot;
import frc.robot.commands.commandgroups.IntakeFuel;
import frc.robot.commands.commandgroups.OuttakeFuel;
import frc.robot.commands.commandgroups.ShootingRoutine;
import frc.robot.subsystems.IntakeRollersSub;
import frc.robot.subsystems.LedSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.IndexerSub;
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
  public final IntakeRollersSub intakeSub = new IntakeRollersSub();
  public final ShooterSub shooterSub = new ShooterSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ConveyorSub conveyorSub = new ConveyorSub();
  public final IndexerSub indexerSub = new IndexerSub();
  public final LedSub ledsub = new LedSub();

  private final PS5Controller driverJoyStick = new PS5Controller(OIConstants.kDriverControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //  Configure the trigger bindings
    configureBindings();

    //  PRE INIT
    SmartDashboard.putData("Reset Gyro Heading: ", new ResetHeadingCMD(swerveSub));

    //  DEFAULT COMMANDS

     // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCMD(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kL3Button),
             /// By default will be on field oriented.
            () -> !
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 

    ledsub.setDefaultCommand(new PeriodicLightsCMD(ledsub));
    
      autoChooser = new SendableChooser<Command>();
    
      autoChooser.setDefaultOption("N/A", Commands.none());
      autoChooser.addOption("MoveBackward", new PathPlannerAuto("MoveBackward"));
      autoChooser.addOption("MiddleDepotShootCenter", new PathPlannerAuto("MiddleDepotShootCenter"));
      autoChooser.addOption("LeftBumpSweep", new PathPlannerAuto("LeftBumpSweep"));
      autoChooser.addOption("RightBumpSweep", new PathPlannerAuto("RightBumpSweep"));
      autoChooser.addOption("LeftBumpToDepotShootMiddleAuto", new PathPlannerAuto("LeftBumpToDepotShootMiddleAuto"));
      autoChooser.addOption("RightBumpFeed", new PathPlannerAuto("RightBumpFeed"));
      autoChooser.addOption("LeftBumpFeed", new PathPlannerAuto("LeftBumpFeed"));
      autoChooser.addOption("LeftBumpSweepToDepotLeftShoot", new PathPlannerAuto("LeftBumpSweepToDepotLeftShoot"));

      SmartDashboard.putData("AutoMode: ", autoChooser);
  }


  private void configureBindings() {

    // RESET HEADING
    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

    new JoystickButton(driverJoyStick, Constants.OIConstants.kR1Button).whileTrue(
      new IntakeFuel(intakeSub, conveyorSub, indexerSub, intakePitcherSub)
    );
    
    new JoystickButton(driverJoyStick, Constants.OIConstants.kL1Button).whileTrue(
      new OuttakeFuel(intakeSub, conveyorSub, indexerSub)
    );

    new JoystickButton(driverJoyStick, OIConstants.kTouchPadButton).whileTrue(
      new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.kPitcherInDegrees)
    );
    new JoystickButton(driverJoyStick, Constants.OIConstants.kR2TriggerButton).whileTrue(
      new ShootingRoutine(shooterSub, indexerSub, conveyorSub, intakePitcherSub, swerveSub)
    );

    Command StopShooting = 
    new ParallelCommandGroup(
      new StopShooterMotorsCMD(shooterSub),
      new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.kPitcherOutDegrees) // when flutter ends, intake may be in
    );

    //when the r2 trigger isnt being held stop the shooter motors and put the intake pitcher out to idle
    new JoystickButton(driverJoyStick, Constants.OIConstants.kR2TriggerButton).whileFalse(
      StopShooting
    );

    new POVButton(driverJoyStick, OIConstants.kDpadUP).whileTrue(
      new IncrementShooterCMD(shooterSub)
    );

    new POVButton(driverJoyStick, OIConstants.kDpadDOWN).whileTrue(
      new DecrementShooterCMD(shooterSub)    
    );

    new JoystickButton(driverJoyStick, OIConstants.kR3Button).whileTrue(
      new LockWheelsCMD(swerveSub)
    );

    //Fire a shot Manually
    new POVButton(driverJoyStick, OIConstants.kDpadRIGHT).whileTrue(
      new FireShot(indexerSub, conveyorSub, intakePitcherSub)    
    );

    new JoystickButton(driverJoyStick, OIConstants.kPsButton).whileTrue(
      StopShooting
    );

  
    new JoystickButton(driverJoyStick, OIConstants.kL2TriggerButton).whileTrue(
      new AlignToTargetCMD(swerveSub, 
        () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis), 
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis)
      )
    );

    NamedCommands.registerCommand("Intake", new IntakeFuel(intakeSub, conveyorSub, indexerSub, 
      intakePitcherSub));
    NamedCommands.registerCommand("ShootingRoutine", new ShootingRoutine(shooterSub, indexerSub, 
      conveyorSub, intakePitcherSub, swerveSub));
    NamedCommands.registerCommand("AlignToTarget", new AlignToTargetCMD(swerveSub, () -> 0.0, () -> 0.0));
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
