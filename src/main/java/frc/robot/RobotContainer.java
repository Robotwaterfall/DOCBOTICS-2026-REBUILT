// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.subsystems.IntakeRollersSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.util.TelemetryManager;

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

  private final PS5Controller driverJoyStick = new PS5Controller(OIConstants.kDriverControllerPort);

  private final TelemetryManager teleManager = new TelemetryManager(Constants.telemetryUpdate);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    //  Configure the trigger bindings
    configureBindings();

    //  register telemetry from different subsystems
    registerSubsystems();

    //  run telemetry
    runTelemetry();

    //  PRE INIT
    autoChooser = AutoBuilder.buildAutoChooser(); //Default auto will be 'Commands.none()'
    SmartDashboard.putData("AutoMode: ", autoChooser);
    SmartDashboard.putData("Reset Gyro Heading: ", new ResetHeadingCMD(swerveSub));

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
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx), 
            () -> driverJoyStick.getRawButton(OIConstants.kLockWheelsButton))); 


    //Intake pitcher should be idle by default
    intakePitcherSub.setDefaultCommand(new MoveIntakePitcherCMD(intakePitcherSub, 0)); 
    
  }


  private void configureBindings() {

    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

   
    
  }

  private void registerSubsystems(){

    teleManager.registerSubsystem("Conveyor: ", conveyorSub);
    teleManager.registerSubsystem("Hood: ", hoodSub);
    teleManager.registerSubsystem("IntakePitcher: ", intakePitcherSub);
    teleManager.registerSubsystem("Intake: ", intakeSub);
    teleManager.registerSubsystem("Shooter: ", shooterSub);

  }

  public void runTelemetry(){
    teleManager.update();
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
