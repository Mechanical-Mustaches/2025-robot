package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeIntakeCommand extends Command{
    private final AlgaeHandlerSubsystem AlgaeSubsystem;
    private ElevatorSubsystem.Level targetLevel;

    public AlgaeIntakerCommand(AlgaeHandlerSubsystem subsystem){
        AlgaeSubsystem = subsystem;
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
