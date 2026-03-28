// package frc.robot.commands.commandgroups;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Constants.IntakePitcherConstants;
// import frc.robot.commands.MoveIntakePitcherCMD;
// import frc.robot.subsystems.IntakePitcherSub;

// public class FlutterIntake extends SequentialCommandGroup{

//     public FlutterIntake(IntakePitcherSub intakePitcherSub){

//         addCommands( 
//             new MoveIntakePitcherCMD(intakePitcherSub, IntakePitcherConstants.intakePitcherFlutterDegrees),
//             new MoveIntakePitcherCMD(intakePitcherSub, Constants.IntakePitcherConstants.kPitcherOutDegrees)
//         );
//     }
// }
