package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSub;

public class RunIndexerCMD extends Command{

    IndexerSub indexerSub;
    double indexPower;

    public RunIndexerCMD(IndexerSub indexerSub, double indexPower){
        this.indexerSub = indexerSub;
        this.indexPower = indexPower;
        addRequirements(indexerSub);
    }

    @Override
    public void initialize(){
        indexerSub.setIndexSpeed(0);
    }

    @Override
    public void execute(){
        indexerSub.setIndexSpeed(indexPower);
    }

    @Override
    public void end(boolean interrupted){
        indexerSub.setIndexSpeed(0);
    }

}