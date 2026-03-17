package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollersSub;

public class IntakeRollerPowerCMD extends Command {

    IntakeRollersSub intakeRollerSub;
    double intakeRollerPower = 0;
  

    // Constructor
    public IntakeRollerPowerCMD(IntakeRollersSub intakeSub, double intakeRollerPower) {
        this.intakeRollerSub = intakeSub;
        this.intakeRollerPower = intakeRollerPower;
       
        addRequirements(intakeSub);
    }

  // Called when the command is initially scheduled. Preps motors.
  @Override
  public void initialize() {
    //stop intake when command initializes
    intakeRollerSub.setMotorPower(0);
    intakeRollerSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled. Calculates the power based on given trigger values
  @Override
  public void execute() {
   
    intakeRollerSub.setMotorPower(intakeRollerPower);
    
  }

  @Override
  public void end(boolean interrupted) {
   
    intakeRollerSub.setMotorPower(0);
    intakeRollerSub.stopMotor();
    
  }

}
