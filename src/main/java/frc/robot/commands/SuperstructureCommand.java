package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;


public class SuperstructureCommand extends Command{
    private final SuperstructureSubsystem superstructure;
    private SuperstructureSubsystem.Stage targetStage;
   

    public SuperstructureCommand(SuperstructureSubsystem subsystem, SuperstructureSubsystem.Stage stage){
        superstructure = subsystem;
        targetStage = stage;
    }

    @Override
    public void initialize() {
        superstructure.toStage(targetStage);
        
    }
    
    
   
    
  
    
     @Override
     public boolean isFinished() {
       return targetStage.equals(superstructure.getStage());
     }
}
