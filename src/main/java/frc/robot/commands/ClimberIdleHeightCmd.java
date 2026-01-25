package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSub;

public class ClimberIdleHeightCmd extends Command{

    public final ClimberSub climberSub;
    public final PIDController climberController;

    public final SparkMax primaryClimbingMotor;
    public final SparkMax secondaryClimbingMotor;

    public ClimberIdleHeightCmd(ClimberSub climberSub){

        this.climberSub = climberSub;

        this.climberController = climberSub.getClimberController();

        this.primaryClimbingMotor = climberSub.getPrimaryClimberMotor();
        this.secondaryClimbingMotor = climberSub.getSecondaryClimberMotor();

        addRequirements(climberSub);

    }


    @Override
    public void initialize(){

        primaryClimbingMotor.set(0);
        primaryClimbingMotor.stopMotor();

        SmartDashboard.putBoolean("idleClimberHeight",true);

    }

    @Override
    public void execute(){

         /* Send elevator telemetry */
        SmartDashboard.putData("elevatorController", climberController);
        SmartDashboard.putNumber("elevatorPositionError_Inches", climberSub.getClimberController().getError());
        SmartDashboard.putNumber("elevatorPosition_Inches", climberSub.getPrimaryClimberPosition());
        
        // Drive climber Motor to set-point based on climber controller.
        //AFTER TESTING CHANGE SETPOINT TO THE VARIABLE SETPOINT IN CLIMBERSUB. 
        double output = climberController.calculate(climberSub.getPrimaryClimberPosition(), climberSub.getClimberSetpoint());

        primaryClimbingMotor.set(output);

    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("idleClimberHeight", false);
        primaryClimbingMotor.set(0);
        primaryClimbingMotor.stopMotor();

    }

    @Override
    public boolean isFinished(){
        
        return false;
    }

}
