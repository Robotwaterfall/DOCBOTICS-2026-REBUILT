package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSub;

public class TeleOpIntakePowerCMD extends Command {

    IntakeSub intakeSub;
    public final Supplier<Boolean> intakeSupplier;
    public final Supplier<Boolean> outakeSupplier;

    // Constructor
    public TeleOpIntakePowerCMD(IntakeSub intakeSub, Supplier<Boolean> intakeSupplier, Supplier<Boolean> outakeSupplier) {
        this.intakeSub = intakeSub;
        this.intakeSupplier = intakeSupplier;
        this.outakeSupplier = outakeSupplier;
        addRequirements(intakeSub);
    }

  // Called when the command is initially scheduled. Preps motors.
  @Override
  public void initialize() {
    //stop intake when command initializes
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled. Calculates the power based on given trigger values
  @Override
  public void execute() {
    //if intake supplier button is pressed intake, otherwise if the outtake button is pressed
    //outtake.
    if(intakeSupplier.get() == true){
      intakeSub.setMotorPower(IntakeConstants.kIntakeMotorPower);
    }else if(outakeSupplier.get() == true){
      intakeSub.setMotorPower(IntakeConstants.kOutakeMotorPower);
    } else{
      intakeSub.setMotorPower(0);
    }
    
  }

}
