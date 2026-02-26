package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakePitcherConstants;
import frc.robot.subsystems.IntakePitcherSub;

public class IdleIntakePitcherCmd extends Command {

  IntakePitcherSub intakePitcherSub;

  // Constructor
  public IdleIntakePitcherCmd(IntakePitcherSub intakePitcherSub) {
    this.intakePitcherSub = intakePitcherSub;
    addRequirements(intakePitcherSub);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePitcherSub.setMotorPower(0);
    intakePitcherSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePitcherSub.movePicherToSetpoint(IntakePitcherConstants.kPitcherOut);
    SmartDashboard.putData(intakePitcherSub);
  }

}
