package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.subsystems.IntakePitcherSub;

public class IdleIntakePitcherCMD extends Command {

  IntakePitcherSub intakePitcherSub;

  // Constructor
  public IdleIntakePitcherCMD(IntakePitcherSub intakePitcherSub) {
    this.intakePitcherSub = intakePitcherSub;
    addRequirements(intakePitcherSub);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //when command starts stop motors and set power 0
    intakePitcherSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //when command executes push pitcher out and idle so that we are ready to pick up fuel
    intakePitcherSub.movePicherToSetpoint(IntakePitcherConstants.kPitcherOut);
    SmartDashboard.putData(intakePitcherSub);
  }

}
