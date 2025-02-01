package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class DumbElevatorCommand extends Command{
    private final ElevatorSubsystem elevator;
    private boolean shouldMoveUp;

    public DumbElevatorCommand(ElevatorSubsystem subsystem, boolean up){
        elevator = subsystem;
        shouldMoveUp = up;
    }

    @Override
    public void initialize() {
        elevator.adjust(shouldMoveUp);
        
        
        
    }
  
   
    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
  
    
    

}
