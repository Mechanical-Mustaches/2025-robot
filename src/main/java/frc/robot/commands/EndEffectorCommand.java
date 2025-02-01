package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffectorCommand extends Command  {
    EndEffectorSubsystem endEffector; 

    public EndEffectorCommand(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }
    @Override
    public void initialize(){
        endEffector.effectorSpin();
    }
    @Override
    public void end(boolean interupt){
       endEffector.effectorStop();
    }
}
