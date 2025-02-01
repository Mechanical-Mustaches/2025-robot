package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class SuperstructureCommand extends Command {
    SuperstructureSubsystem superstructureSubsystem;

    public SuperstructureCommand(SuperstructureSubsystem superstructureSubsystem){
    this.superstructureSubsystem = superstructureSubsystem;
    }
    
    @Override
    public void initialize(){
        superstructureSubsystem.pivotOut();
    }
   
    @Override
    public void end(boolean interupt){
       superstructureSubsystem.pivotStop();
    }
}
