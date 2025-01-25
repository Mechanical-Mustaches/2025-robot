package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command{
    private final ElevatorSubsystem elevator;
    private ElevatorSubsystem.Level targetLevel;

    public ElevatorCommand(ElevatorSubsystem subsystem, ElevatorSubsystem.Level level){
        elevator = subsystem;
        targetLevel = level;
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetLevel);
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
