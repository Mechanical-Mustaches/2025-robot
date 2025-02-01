package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;


public class AlgaeIntakeCommand extends Command{
    private AlgaeHandlerSubsystem AlgaeSubsystem;
    

    public AlgaeIntakeCommand(AlgaeHandlerSubsystem subsystem){
        AlgaeSubsystem = subsystem;
    }

    @Override
    public void initialize() {
       AlgaeSubsystem.intake();
    }
  
   
    @Override
    public void end(boolean interrupted) {
        AlgaeSubsystem.stopIntake();
    }
  
    


}
