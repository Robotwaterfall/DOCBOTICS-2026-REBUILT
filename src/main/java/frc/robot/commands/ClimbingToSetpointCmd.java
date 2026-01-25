package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSub;

public class ClimbingToSetpointCmd extends Command{

    public final ClimberSub climberSub;
    public final PIDController climberController;

    public final SparkMax primaryClimbingMotor;
    public final SparkMax secondaryClimbingMotor;

    public final double climbingHeightSetpoint_meters;

    public ClimbingToSetpointCmd(ClimberSub climberSub, double climbingHeightSetpoint_meters){

        this.climberSub = climberSub;

        this.climberController = climberSub.getClimberController();

        this.primaryClimbingMotor = climberSub.getPrimaryClimberMotor();
        this.secondaryClimbingMotor = climberSub.getSecondaryClimberMotor();

        this.climbingHeightSetpoint_meters = climbingHeightSetpoint_meters;

        addRequirements(climberSub);

    }


    @Override
    public void initialize(){

        primaryClimbingMotor.set(0);
        primaryClimbingMotor.stopMotor();

        climberSub.setClimberSetpoint(climbingHeightSetpoint_meters);
        SmartDashboard.putBoolean("isClimbingToSetpoint", true);

    }

    @Override
    public void execute(){

         /* Send elevator telemetry */
        SmartDashboard.putData(climberController);
        SmartDashboard.putNumber("elevatorPositionError_Inches", climberSub.getClimberController().getError());
        SmartDashboard.putNumber("elevatorPosition_Inches", climberSub.getPrimaryClimberPosition());
        
        // Drive climber Motor to set-point based on climber controller.
        //AFTER TESTING CHANGE SETPOINT TO THE VARIABLE SETPOINT IN CLIMBERSUB. 
        double output = climberController.calculate(climberSub.getPrimaryClimberPosition(), climbingHeightSetpoint_meters);

        primaryClimbingMotor.set(output);

    }

    @Override
    public void end(boolean interupted){
        SmartDashboard.putBoolean("isClimbingToSetpoint", false);
        primaryClimbingMotor.set(0);
        primaryClimbingMotor.stopMotor();

    }

    @Override
    public boolean isFinished(){
        if (Math.abs(climberController.getError()) <= ClimberConstants.climberTolerance) {
            return true;
        }
        return false;
    }

}
