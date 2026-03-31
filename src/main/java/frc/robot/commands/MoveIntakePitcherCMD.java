package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePitcherSub;

public class MoveIntakePitcherCMD extends Command{

    IntakePitcherSub intakePitcherSub;

    double intakePitcherPosDeg;

    // Constructor
  public MoveIntakePitcherCMD(IntakePitcherSub intakePitcherSub, double intakePitcherPosDeg) {
    this.intakePitcherSub = intakePitcherSub;
    this.intakePitcherPosDeg = intakePitcherPosDeg;
    addRequirements(intakePitcherSub);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakePitcherSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //move intake pitcher to deg
    intakePitcherSub.setPitcherAngle(intakePitcherPosDeg); 
  }

  @Override
  public boolean isFinished(){
    //when the pitcher is at the set setpoint stop command
    return intakePitcherSub.isPitcherAtSetpoint();

  }

}