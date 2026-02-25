package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSub;

public class AutoIntakePowerCmd extends Command {

    IntakeSub intakeSub;
    Timer elappsedTime;
    double totalTime;

    public AutoIntakePowerCmd(IntakeSub intakeSub, double totalTime) {
        this.intakeSub = intakeSub;
        this.elappsedTime = new Timer();
        this.totalTime = totalTime;
        addRequirements(intakeSub);
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
    elappsedTime.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSub.setMotorPower(Constants.IntakeConstants.kIntakeMotorPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elappsedTime.get() >= totalTime;
  }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
  }

}
