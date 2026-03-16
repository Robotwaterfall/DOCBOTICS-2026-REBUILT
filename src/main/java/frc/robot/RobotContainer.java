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
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.TeleOpIntakePowerCMD;
import frc.robot.commands.AdaptableShooterCMD;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.subsystems.IntakeRollerSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.AutoIntakePowerCMD;
import frc.robot.commands.HoodServoAdjustCMD;
import frc.robot.commands.IdleIntakePitcherCMD;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.SwerveSub;
import frc.util.TelemetryManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final SwerveSub swerveSub = new SwerveSub();
  public final IntakeRollerSub intakeSub = new IntakeRollerSub();
  public final HoodSub hoodSub = new HoodSub();
  public final ShooterSub shooterSub = new ShooterSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ConveyorSub conveyorSub = new ConveyorSub();

  private final XboxController driverJoyStick = new XboxController(OIConstants.kDriverControllerPort);

  private final TelemetryManager teleManager = new TelemetryManager(Constants.telemetryUpdate);

  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //register telemetry from different subsystems
    registerSubsystems();

    //run telemetry
    runTelemetry();

    

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

    
    intakeSub.setDefaultCommand(new TeleOpIntakePowerCMD(intakeSub, 
      () -> driverJoyStick.getRightBumperButtonPressed(), 
      () -> driverJoyStick.getLeftBumperButtonPressed())); 
    
    conveyorSub.setDefaultCommand(new RunConveyorCMD(
      conveyorSub, shooterSub, 
      () -> driverJoyStick.getLeftBumperButtonPressed()));
    
    

    hoodSub.setDefaultCommand(
      new HoodServoAdjustCMD(hoodSub) //Hood should constantly be adjusting if april tag is detected
    );

    //Intake pitcher should be idle by default, but can be moved to different angles when the MoveIntakePitcherCmd is scheduled
    intakePitcherSub.setDefaultCommand(new IdleIntakePitcherCMD(intakePitcherSub)); 

    autoChooser = AutoBuilder.buildAutoChooser(); //Default auto will be 'Commands.none()'
    SmartDashboard.putData("AutoMode: ", autoChooser);
    
  }


  private void configureBindings() {

    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

    //This command runs the shooter routine
    Command adaptableShootCommand = 
      new ParallelDeadlineGroup(
        
      new WaitCommand(autoConstants.timeElapsedShootingSecounds),
        
      new ParallelCommandGroup(

          new AdaptableShooterCMD(shooterSub, hoodSub,() -> 1.0, true),
          new HoodServoAdjustCMD(hoodSub),

          new SequentialCommandGroup(
            new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.kPitcherIn),
            new WaitCommand(autoConstants.timeBetweenPitcherInAndOut),
            new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.kPitcherOut)
          ).repeatedly()

        ));
    NamedCommands.registerCommand("adaptableShooterCommand", adaptableShootCommand);

    Command intakeFor5Secounds = 
      new SequentialCommandGroup(
        new AutoIntakePowerCMD(intakeSub, 5)
      );
    NamedCommands.registerCommand("intakeFor5Secounds", intakeFor5Secounds);



  
  
    
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
