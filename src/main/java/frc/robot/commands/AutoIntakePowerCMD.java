package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class AutoIntakePowerCMD extends Command {

    IntakeSub intakeSub;
    SparkMax intakeMotor;

    public AutoIntakePowerCMD(IntakeSub intakeSub) {
        this.intakeSub = intakeSub;
        this.intakeMotor = intakeSub.getIntakeMotor();
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSub.setMotorPower(0);
    intakeMotor.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
