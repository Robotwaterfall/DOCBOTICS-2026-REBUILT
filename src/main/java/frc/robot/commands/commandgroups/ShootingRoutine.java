package frc.robot.commands.commandgroups;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignToHubCMD;
import frc.robot.subsystems.ConveyorSub;
import frc.robot.subsystems.HoodSub;
import frc.robot.subsystems.IndexerSub;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.SwerveSub;

public class ShootingRoutine extends SequentialCommandGroup{

    public ShootingRoutine(
        SwerveSub swerveSub,
        ShooterSub shooterSub,
        HoodSub hoodSub,
        IndexerSub indexerSub,
        ConveyorSub conveyorSub,
        IntakePitcherSub intakePitcherSub,
        PS5Controller driverJoystick
    ){
        //Make a instance of the align to hub command
        AlignToHubCMD alignToHubCMD = new AlignToHubCMD(swerveSub);

        /**  Phase 1: Run PrepareShot + AlignToHub in parallel while waiting for the shooter
             to reach its target speed AND the robot to be aligned. The WaitForShooterReady
             command is the *deadline*, meaning this whole group ends only when the shooter
             is ready. Until then, PrepareShot continuously updates shooter RPM/hood angle,
             and AlignToHubCMD keeps rotating the robot toward the target.
        */  
        Command WaitForReady = 
            new WaitForShooterReady(shooterSub, alignToHubCMD)
                .deadlineWith(
                    new PrepareShot(
                        shooterSub, 
                        hoodSub, 
                        swerveSub),
                        alignToHubCMD
                );
        /**  Phase 2: Begin feeding game pieces into the shooter. FireShot is the *deadline*
             command, meaning this parallel group continues running only while FireShot is
             active (typically as long as the driver holds the shoot button). Meanwhile,
             PrepareShot keeps updating shooter RPM and hood angle, and AlignToHubCMD keeps
             the robot aimed at the target throughout the entire firing sequence.
        */

        Command Fire = 
            new FireShot(indexerSub, conveyorSub, intakePitcherSub, driverJoystick)
                .deadlineWith(
                    new PrepareShot(
                        shooterSub, 
                        hoodSub, 
                        swerveSub),
                        alignToHubCMD
                );


        
        // Schedule the two phases sequentially
        addCommands(
            WaitForReady,
            Fire
        );
                    

    }

}
