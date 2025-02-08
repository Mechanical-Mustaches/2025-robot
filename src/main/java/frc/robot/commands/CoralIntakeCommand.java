package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralIntakeCommand extends Command {
    EndEffectorSubsystem endEffector;

    public CoralIntakeCommand(EndEffectorSubsystem endEffector){
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
    @Override
    public boolean isFinished(){
        return !endEffector.isCoralSeenBack() && endEffector.isCoralSeenFront();
    }
}
