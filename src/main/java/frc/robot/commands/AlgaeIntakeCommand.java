package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;


public class AlgaeIntakeCommand extends Command{
    private AlgaeHandlerSubsystem algaeSubsystem;
    

    public AlgaeIntakeCommand(AlgaeHandlerSubsystem subsystem){
        algaeSubsystem = subsystem;
    }

    @Override
    public void initialize() {
       algaeSubsystem.intake();
    }
  
   
    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopIntake();
    }
  
    
    

}
