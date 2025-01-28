package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class VerticleClimberCommand extends Command{
    ClimberSubsystem climber;

    public VerticleClimberCommand(ClimberSubsystem climber){
    this.climber = climber;
    }
    @Override
    public void initialize(){
        climber.verticleClimber();
    }
   
    @Override
    public void end(boolean interupt){
       climber.climberStop();
    }
}
