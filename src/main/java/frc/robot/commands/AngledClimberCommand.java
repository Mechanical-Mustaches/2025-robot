package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class AngledClimberCommand extends Command{
    ClimberSubsystem climber;

    public AngledClimberCommand(ClimberSubsystem climber){
    this.climber = climber;
    }
    @Override
    public void initialize(){
        climber.climber2();
    }
   
     @Override
     public void end(boolean interupt){
        climber.climberStop();
     }

    // @Override
    // public boolean isFinished(){
    //     if(climber.isDone()){
    //         climber.climberStop();
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }
}
