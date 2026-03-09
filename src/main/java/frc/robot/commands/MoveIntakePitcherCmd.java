package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePitcherSub;

public class MoveIntakePitcherCmd extends Command{

    IntakePitcherSub intakePitcherSub;

    double intakePitcherPosDeg;

    // Constructor
  public MoveIntakePitcherCmd(IntakePitcherSub intakePitcherSub, double intakePitcherPosDeg) {
    this.intakePitcherSub = intakePitcherSub;
    this.intakePitcherPosDeg = intakePitcherPosDeg;
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
    intakePitcherSub.movePicherToSetpoint(intakePitcherPosDeg);
 
  }


  @Override
  public boolean isFinished(){
    return intakePitcherSub.getIntakePitchSetpoint_degrees() == intakePitcherPosDeg;

  }

}
