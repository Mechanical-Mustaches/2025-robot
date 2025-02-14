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
        climber.angledClimber();
    }
   
    @Override
    public void end(boolean interupt){
       climber.climberStop();
    }
}
