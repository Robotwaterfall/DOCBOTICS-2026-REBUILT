package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSub;

public class TeleOpIntakePowerCMD extends Command {

    IntakeSub intakeSub;
    public final Supplier<Double> intakePowerSpeedSupplier;
    public final Supplier<Double> outakePowerSpeedSupplier;

    // Constructor
    public TeleOpIntakePowerCMD(IntakeSub intakeSub, Supplier<Double> intakePowerSpeedSupplier, Supplier<Double> outakePowerSpeedSupplier) {
        this.intakeSub = intakeSub;
        this.intakePowerSpeedSupplier = intakePowerSpeedSupplier;
        this.outakePowerSpeedSupplier = outakePowerSpeedSupplier;
        addRequirements(intakeSub);
    }

  // Called when the command is initially scheduled. Preps motors.
  @Override
  public void initialize() {
    intakeSub.setMotorPower(0);
    intakeSub.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled. Calculates the power based on given trigger values
  @Override
  public void execute() {
    intakeSub.setMotorPower((intakePowerSpeedSupplier.get() - outakePowerSpeedSupplier.get()) * 0.3);
  }

}
