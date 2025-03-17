package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;


public class AlgaeIntakeCommand extends Command{
    private AlgaeHandlerSubsystem algaeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    ElevatorSubsystem.Level level;
    public static ElevatorSubsystem.Level algaeLevel;
    

    public AlgaeIntakeCommand(AlgaeHandlerSubsystem algaeSubsystem, ElevatorSubsystem elevatorSubsystem, ElevatorSubsystem.Level level){
        this.algaeSubsystem = algaeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.level = level;
    }

    @Override
    public void initialize() {
       algaeSubsystem.intake();
       if (10 < elevatorSubsystem.getEncoderValue() && elevatorSubsystem.getEncoderValue() < 13){
            algaeLevel = Level.L2Algae;
       }
       if (17.5 < elevatorSubsystem.getEncoderValue() && elevatorSubsystem.getEncoderValue() < 20.5){
            algaeLevel = Level.L3Algae;
       }
    }
  
   
    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.stopIntake();
    }
  
    
    

}
