// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.PartialResultException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ConveyorConstant;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.RunConveyorCMD;
import frc.robot.commands.RunIndexerCMD;
import frc.robot.commands.RunShooterCMD;
import frc.robot.commands.StopShooterMotorsCMD;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.SwerveLimelightLockCMD;
import frc.robot.commands.commandgroups.FireShot;
import frc.robot.commands.commandgroups.IntakeFuel;
import frc.robot.commands.commandgroups.OuttakeFuel;
import frc.robot.subsystems.IntakeRollersSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.commands.AdjustHoodCMD;
import frc.robot.commands.AlignToHubCMD;
import frc.robot.commands.DecrementHoodCMD;
import frc.robot.commands.DecrementShooterCMD;
import frc.robot.commands.IncrementHoodCMD;
import frc.robot.commands.IncrementShooterCMD;
import frc.robot.commands.MoveIntakePitcherCMD;
import frc.robot.commands.ShooterTuningCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
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
  public final HoodSub hoodSub = new HoodSub();
  public final ShooterSub shooterSub = new ShooterSub();
  public final IntakePitcherSub intakePitcherSub = new IntakePitcherSub();
  public final ConveyorSub conveyorSub = new ConveyorSub();
  public final IndexerSub indexerSub = new IndexerSub();

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
            () -> driverJoyStick.getRawButtonPressed(OIConstants.kSlowModeIdx),
             /// By default will be on field oriented.
            () -> !
            driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 
    

    // intakePitcherSub.setDefaultCommand(
    //   new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOutDegrees)
    // );

    // telemetrySub.setDefaultCommand(
    //   new TelemetryManagerCMD(swerveSub)
    // );

    
      autoChooser = new SendableChooser<Command>();
      // autoChooser.addOption("BackUpShootMiddle", new PathPlannerAuto("MoveBackShoot"));

      // autoChooser.addOption("MiddleDepotShootCenter", 
      //                       new PathPlannerAuto("MiddleDepotShootCenter"));
     
      // autoChooser.addOption("RightBumpSweep", new PathPlannerAuto("RightBumpSweep"));
      // autoChooser.addOption("LeftBumpSweep", new PathPlannerAuto("LeftBumpSweep"));

      // autoChooser.addOption("LeftBumpToDepotShootMiddleAuto", new PathPlannerAuto("LeftBumpToDepotShootMiddleAuto"));
      autoChooser.addOption("MoveBackward", new PathPlannerAuto("MoveBackward"));

      SmartDashboard.putData("AutoMode: ", autoChooser);
  }


  private void configureBindings() {

    // RESET HEADING
    new JoystickButton(driverJoyStick, OIConstants.kDriveGyroResetButtonIdx).whileTrue(
      new ResetHeadingCMD(swerveSub)
    );

    new JoystickButton(driverJoyStick, OIConstants.kShootingRoutineButton).whileTrue(
      new FireShot(indexerSub, conveyorSub, intakePitcherSub)
    );

    // new JoystickButton(driverJoyStick, OIConstants.kTouchPadButton).whileTrue(
    //   new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherInDegrees)
    // );

    


    // // INTAKE
    // new JoystickButton(driverJoyStick, OIConstants.kIntakeButton).whileTrue(
    //   new IntakeFuel(intakeSub, conveyorSub, indexerSub, intakePitcherSub)
    // );

    // // OUTTAKE
    // new JoystickButton(driverJoyStick, OIConstants.kOuttakeButton).whileTrue(
    //   new OuttakeFuel(intakeSub, conveyorSub, indexerSub)
    // );

    // // INCREMENT SHOOTER MANUALLY
    // POVButton incShooterButton = new POVButton(driverJoyStick, Constants.OIConstants.kDpadRIGHT);
    // incShooterButton.onTrue(new IncrementShooterCMD(shooterSub, ShooterConstants.shooterVelocityPlusPerPress));
    
    // // DECREMENT SHOOTER MANUALLY
    // POVButton decShooterButton = new POVButton(driverJoyStick, Constants.OIConstants.kDpadLEFT);
    // decShooterButton.onTrue(new DecrementShooterCMD(shooterSub, ShooterConstants.shooterVelocityPlusPerPress));

    // // INCREMENT HOOD MANUALLY
    // POVButton incHoodButton = new POVButton(driverJoyStick, Constants.OIConstants.kDpadUP);
    // incHoodButton.onTrue(new IncrementHoodCMD(hoodSub, HoodConstants.hoodAnglePlusPerPress));

    // // DECREMENT HOOD MANUALLY
    // POVButton decHoodButton = new POVButton(driverJoyStick, Constants.OIConstants.kDpadDOWN);
    // decHoodButton.onTrue(new DecrementHoodCMD(hoodSub, HoodConstants.hoodAnglePlusPerPress));

    //STOP ALL SHOOTER MOTORS
    JoystickButton stopShooterMotorsButton = new JoystickButton(driverJoyStick, OIConstants.kPsButton);
    stopShooterMotorsButton.onTrue(new StopShooterMotorsCMD(shooterSub));

    // Command shootClose =
    // new ParallelCommandGroup(
    //   new RunShooterCMD(shooterSub)
    // );

    // new JoystickButton(driverJoyStick, Constants.GeorgianCollegeConstants.kCloseShotButton).whileTrue(
    //   shootClose
    // );

    // Command shootFar =
    // new ParallelCommandGroup(
    //   new RunShooterCMD(shooterSub, Constants.GeorgianCollegeConstants.kShootFarVelocity),
    //   new AdjustHoodCMD(hoodSub, Constants.GeorgianCollegeConstants.kShootFarAngle)
    // );
    // new JoystickButton(driverJoyStick, Constants.GeorgianCollegeConstants.kFarShotButton).whileTrue(
    //   shootFar
    // );

    // // TUNING MODE: Hold button 3, D-pad LEFT/RIGHT adjusts shooter velocity
    // new JoystickButton(driverJoyStick, Constants.GeorgianCollegeConstants.kNeutralShotButton).whileTrue(
    //   new ShooterTuningCMD(shooterSub, driverJoyStick)
    // );

    // Command fireShot = 
    // new ParallelCommandGroup(
    //   new RunIndexerCMD(indexerSub, ShooterConstants.kIndexSpeed),
    //   new RunConveyorCMD(conveyorSub, ConveyorConstant.conveyorPower)

    // );
    // new JoystickButton(driverJoyStick, Constants.OIConstants.kShootingRoutineButton).whileTrue(
    //   fireShot
    // );

    
    // LIMELIGHT LOCK / AIM ASSIST
    new JoystickButton(driverJoyStick, OIConstants.kPrepareShotButton).whileTrue(
      new SwerveLimelightLockCMD(
        swerveSub,
        () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
        DriveConstants.autoTargetConstants.autoOrientSpeed
      )
    );



    // NamedCommands.registerCommand("Intake", new IntakeFuel(intakeSub, conveyorSub, indexerSub, intakePitcherSub));
    // NamedCommands.registerCommand("IntakeOut", new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOutDegrees));
    // NamedCommands.registerCommand("LockOnHub", new SwerveLimelightLockCMD(
    //                                 swerveSub, 
    //                                   () -> 0.0, 
    //                                   () -> 0.0, 
    //                                    DriveConstants.autoTargetConstants.autoOrientSpeed));
    // NamedCommands.registerCommand("shootClose", shootClose);
    // NamedCommands.registerCommand("shootFar", shootFar);
    // NamedCommands.registerCommand("fireShot", fireShot);



  

   
    
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return null;
  }
}