package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollersSub;

public class RunIntakeRollersCMD extends Command {

    IntakeRollersSub intakeRollerSub;
    double intakeRollerVelocityRPS = 0;
  

    // Constructor
    public RunIntakeRollersCMD(IntakeRollersSub intakeSub, double intakeRollerVelocityRPS) {
        this.intakeRollerSub = intakeSub;
        this.intakeRollerVelocityRPS = intakeRollerVelocityRPS;
       
        addRequirements(intakeSub);
    }

  // Called when the command is initially scheduled. Preps motors.
  @Override
  public void initialize() {
    intakeRollerSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled. Calculates the power based on given trigger values
  @Override
  public void execute() {
   
    intakeRollerSub.setVelocity(intakeRollerVelocityRPS);
    
  }

  @Override
  public void end(boolean interrupted) {
   
    intakeRollerSub.stopMotor();
    
  }

}
