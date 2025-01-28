package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EndEffector3Command extends Command {
    EndEffectorSubsystem endEffector;

    public EndEffector3Command(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }
    @Override
    public void initialize(){
        endEffector.effectorSpin3();
    }
   
    @Override
    public void end(boolean interupt){
       endEffector.effectorStop();
    }
}
