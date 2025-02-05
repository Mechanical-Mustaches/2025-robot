package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class SuperstructureStageOneCommand extends Command{
    private SuperstructureSubsystem superstructureSubsystem;


    public SuperstructureStageOneCommand(SuperstructureSubsystem subsystem){
        this.superstructureSubsystem = subsystem;
    }

    @Override
    public void initialize() {
        
    }
  
   
    @Override
    public void end(boolean interrupted) {
        
    }
}
