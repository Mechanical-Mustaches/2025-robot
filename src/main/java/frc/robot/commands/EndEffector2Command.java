package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffector2Command extends Command {
    EndEffectorSubsystem endEffector;

    public EndEffector2Command(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }
    @Override
    public void initialize(){
        endEffector.effectorSpin2();
    }
   
    @Override
    public void end(boolean interupt){
       endEffector.effectorStop();
    }
}
