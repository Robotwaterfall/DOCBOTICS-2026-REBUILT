// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.JobState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.ResetPoseCMD;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.commands.RunIndexerCMD;
import frc.robot.commands.RunShooterCMD;
import frc.robot.commands.StopShooterMotorsCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.telemetryManagerCMD;
import frc.robot.commands.commandgroups.FlutterIntake;
import frc.robot.commands.commandgroups.Juggle;
import frc.robot.commands.commandgroups.OuttakeFuel;
import frc.robot.commands.commandgroups.PrepareShot;
import frc.robot.commands.commandgroups.ShootingRoutine;
import frc.robot.subsystems.IntakeRollersSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.AdjustHoodCMD;
import frc.robot.commands.DecrementHoodCMD;
import frc.robot.commands.DecrementShooterCMD;
import frc.robot.commands.IncrementHoodCMD;
import frc.robot.commands.IncrementShooterCMD;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.PoseManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveSub swerveSub = new SwerveSub();
  public final IntakeRollersSub intakeSub = new IntakeRollersSub();
  public final HoodSub hoodSub = new HoodSub();
  public final ShooterSub shooterSub = new ShooterSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ConveyorSub conveyorSub = new ConveyorSub();
  public final IndexerSub indexerSub = new IndexerSub();

  private final PS5Controller driverJoyStick = new PS5Controller(OIConstants.kDriverControllerPort);

  // private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    //  Configure the trigger bindings
    configureBindings();

    //  PRE INIT
    // autoChooser = AutoBuilder.buildAutoChooser(); //Default auto will be 'Commands.none()'
    // SmartDashboard.putData("AutoMode: ", autoChooser);
    SmartDashboard.putData("Reset Gyro Heading: ", new ResetHeadingCMD(swerveSub));
    SmartDashboard.putData("Reset Pose: ", new ResetPoseCMD(swerveSub));

    //  DEFAULT COMMANDS

     // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCMD(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kSlowModeIdx),
             /// By default will be on field oriented.
            () -> !
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 

    intakePitcherSub.setDefaultCommand(
      new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOutDegrees)
    );
  }


  private void configureBindings() {

    // RESET HEADING
    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

    // SHOOTING ROUTINE
    // new JoystickButton(driverJoyStick, OIConstants.kShootingRoutineButton).whileTrue(
    //   new ShootingRoutine(swerveSub, shooterSub, hoodSub, indexerSub, conveyorSub, intakePitcherSub, driverJoyStick)
    // );

    // // PREPARE SHOT
    // new JoystickButton(driverJoyStick, OIConstants.kPrepareShotButton).whileTrue(
    //   new PrepareShot(shooterSub, hoodSub, swerveSub)
    // );

    new JoystickButton(driverJoyStick, OIConstants.kShootingRoutineButton).whileTrue(
      new RunShooterCMD(shooterSub, swerveSub, ShooterConstants.kShooterVelocityFps)
    );

    new JoystickButton(driverJoyStick, OIConstants.kPrepareShotButton).whileTrue(
      new RunIndexerCMD(indexerSub, 1.0)
    );

    new JoystickButton(driverJoyStick, OIConstants.kPrepareShotButton).whileTrue(
      new RunConveyorCMD(conveyorSub, 0.6)
    );

    new JoystickButton(driverJoyStick, OIConstants.kPrepareShotButton).whileTrue(
      new FlutterIntake(intakePitcherSub).repeatedly()
    );

    // // JUGGLE WHILE INTAKING
    new JoystickButton(driverJoyStick, OIConstants.kJuggleButton).whileTrue(
      new Juggle(shooterSub, indexerSub, swerveSub, intakeSub, conveyorSub, hoodSub)
    );

    // // OUTTAKE
    new JoystickButton(driverJoyStick, OIConstants.kOuttakeButton).whileTrue(
      new OuttakeFuel(intakeSub, conveyorSub, indexerSub)
    );


    // new JoystickButton(driverJoyStick, OIConstants.kShootingRoutineButton).whileTrue(
    //   new AlignToHubCMD(swerveSub)
    // );

    // INCREMENT SHOOTER MANUALLY
    JoystickButton incShooterButton = new JoystickButton(driverJoyStick, 1);
    incShooterButton.onTrue(new IncrementShooterCMD(shooterSub, ShooterConstants.shooterVelocityPlusPerPress));
    
    // DECREMENT SHOOTER MANUALLY
    JoystickButton decShooterButton = new JoystickButton(driverJoyStick, 3);
    decShooterButton.onTrue(new DecrementShooterCMD(shooterSub, ShooterConstants.shooterVelocityPlusPerPress));

    // INCREMENT HOOD MANUALLY
    JoystickButton incHoodButton = new JoystickButton(driverJoyStick, 4);
    incHoodButton.onTrue(new IncrementHoodCMD(hoodSub, HoodConstants.hoodAnglePlusPerPress));

    // DECREMENT HOOD MANUALLY
    JoystickButton decHoodButton = new JoystickButton(driverJoyStick, 2);
    decHoodButton.onTrue(new DecrementHoodCMD(hoodSub, HoodConstants.hoodAnglePlusPerPress));

    JoystickButton stopMotorsButton = new JoystickButton(driverJoyStick, 13);
    stopMotorsButton.onTrue(new StopShooterMotorsCMD(shooterSub));

    // // MANUAL FIRE SHOT
    // POVButton fireManualShot = new POVButton(driverJoyStick, OIConstants.kDpadRIGHTDOWN);
    // fireManualShot.onTrue(new FireShot(indexerSub, conveyorSub, intakePitcherSub, driverJoyStick));

   
    
  }


  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
