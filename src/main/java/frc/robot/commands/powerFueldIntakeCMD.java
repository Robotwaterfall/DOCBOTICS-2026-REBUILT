package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePitcherSub;
import frc.robot.subsystems.IntakeSub;


public class PowerFueldIntakeCMD extends Command {
    private final IntakeSub intakeSub;
    private final SparkMax intakeConsumerMotor;
    public final Supplier<Double> intakeConsumerSpeedSupplier;
    public final Supplier<Double> outakeConsumerSpeedSupplier;


    public PowerFueldIntakeCMD(
        IntakeSub intakeSub, 
        Supplier<Double> intakeConsumerSpeedSupplier,
        Supplier<Double> outakeConsumerSpeedSupplier) {

        this.intakeSub = intakeSub;
        this.intakeConsumerMotor = intakeSub.getIntakeConsumerMotor();
        this.intakeConsumerSpeedSupplier = intakeConsumerSpeedSupplier;
        this.outakeConsumerSpeedSupplier = outakeConsumerSpeedSupplier;
        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();
    

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        /* power intake consumer motor */
        intakeConsumerMotor.set( 
            (intakeConsumerSpeedSupplier.get() * 0.3) - 
        (outakeConsumerSpeedSupplier.get() * 0.3) );
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();
        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);

    }

    @Override
    public boolean isFinished() {

        return false;
    }
}