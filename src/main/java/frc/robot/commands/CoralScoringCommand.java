package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralScoringCommand extends Command  {
    EndEffectorSubsystem endEffector; 

    public CoralScoringCommand(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }
    @Override
    public void initialize(){
        endEffector.effectorScore();
    }
    @Override
    public void end(boolean interupt){
       endEffector.effectorStop();
    }
    @Override
    public boolean isFinished(){
        return !endEffector.isCoralSeenFront();
    }
}
