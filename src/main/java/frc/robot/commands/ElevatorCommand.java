package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ElevatorCommand extends Command{
    private final ElevatorSubsystem elevator;
    private ElevatorSubsystem.Level targetLevel;
    EndEffectorSubsystem endEffector; 
    private boolean isIntakingAlgae;

    public ElevatorCommand(ElevatorSubsystem subsystem, ElevatorSubsystem.Level level, EndEffectorSubsystem endEffector, boolean isIntakingAlgae){
        elevator = subsystem;
        targetLevel = level;
        this.endEffector = endEffector;
        this.isIntakingAlgae = isIntakingAlgae;
    }

    @Override
    public void initialize() {
        if(!isIntakingAlgae){
        elevator.setPosition(targetLevel, endEffector);
        } else{
            elevator.algaeDescent();
        }
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
