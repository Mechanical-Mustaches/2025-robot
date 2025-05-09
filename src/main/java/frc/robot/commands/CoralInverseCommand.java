package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralInverseCommand extends Command {
    EndEffectorSubsystem endEffector;

    public CoralInverseCommand(EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
    }
    @Override
    public void initialize(){
        endEffector.effectorInverse();
    }
    @Override
    public void end(boolean interupt){
       endEffector.effectorStop();
    }
    @Override
    public boolean isFinished(){
        return endEffector.isCoralSeenBack();
    }
}
