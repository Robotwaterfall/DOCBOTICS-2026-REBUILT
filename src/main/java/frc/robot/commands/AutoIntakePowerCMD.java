package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;

public class AutoIntakePowerCMD extends Command {

  IntakeSub intakeSub;
  Timer elapsedTime;
  double totalTime;

  public AutoIntakePowerCMD(IntakeSub intakeSub, double totalTime) {
    this.intakeSub = intakeSub;
    this.elapsedTime = new Timer();
    this.totalTime = totalTime;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
    elapsedTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setMotorPower(Constants.IntakeConstants.kIntakeMotorPower);
    SmartDashboard.putData(intakeSub);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //finish when time gets to target time
    return elapsedTime.get() >= totalTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop motors and set power 0
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
  }

}
