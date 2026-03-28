// package frc.robot.commands.commandgroups;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AlignToHubCMD;
// import frc.robot.subsystems.ConveyorSub;
// import frc.robot.subsystems.HoodSub;
// import frc.robot.subsystems.IndexerSub;
// import frc.robot.subsystems.IntakePitcherSub;
// import frc.robot.subsystems.ShooterSub;
// import frc.robot.subsystems.SwerveSub;

// public class ShootingRoutine extends SequentialCommandGroup{

//     public ShootingRoutine(
//     SwerveSub swerveSub,
//     ShooterSub shooterSub,
//     HoodSub hoodSub,
//     IndexerSub indexerSub,
//     ConveyorSub conveyorSub,
//     IntakePitcherSub intakePitcherSub
// ) {
//     // Phase 1: Fresh instances for each group
//     Command WaitForReady = 
//         new WaitForShooterReady(shooterSub, new AlignToHubCMD(swerveSub))  // Fresh here
//             .deadlineWith(
//                 new PrepareShot(shooterSub, hoodSub, swerveSub),
//                 new AlignToHubCMD(swerveSub)  // Fresh instance
//             );
    
//     // Phase 2: Fresh instances again
//     Command Fire = 
//         new FireShot(indexerSub, conveyorSub, intakePitcherSub)
//             .deadlineWith(
//                 new PrepareShot(shooterSub, hoodSub, swerveSub),
//                 new AlignToHubCMD(swerveSub)  // Fresh instance (not reused)
//             );

//     addCommands(WaitForReady, Fire);
// }


// }
