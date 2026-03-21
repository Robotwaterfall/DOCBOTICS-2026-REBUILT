package frc.robot.commands;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;

public class RumbleControllerCMD extends Command{

    PS5Controller driverJoystick;

    public RumbleControllerCMD(PS5Controller driverJoystick){
        this.driverJoystick = driverJoystick;
    }
    
    @Override
    public void execute(){
        driverJoystick.setRumble(RumbleType.kBothRumble, OIConstants.kRumblePwr);
    }

    @Override
    public void end(boolean interrupted){
        driverJoystick.setRumble(RumbleType.kBothRumble, 0);
    }

}
