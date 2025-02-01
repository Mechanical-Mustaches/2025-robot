package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;


public class AlgaeIntakeCommand extends Command{
    private AlgaeHandlerSubsystem AlgaeSubsystem;
    

    public void AlgaeIntakerCommand(AlgaeHandlerSubsystem subsystem){
        AlgaeSubsystem = subsystem;
    }

    @Override
    public void initialize() {
       AlgaeSubsystem.intake();
    }
  
   
    @Override
    public void end(boolean interrupted) {
        //elevator.stopElevator();
    }
  
    
    @Override
    public boolean isFinished() {
      return true;
        //TODO: return value based on encoder value, especially for sequential commands.
    }

}
