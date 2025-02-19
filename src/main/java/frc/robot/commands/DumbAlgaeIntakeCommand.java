package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaeIntakeCommand extends Command{
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
   

    public DumbAlgaeIntakeCommand(AlgaeHandlerSubsystem subsystem){
        algaeHandlerSubsystem = subsystem;
    }

    @Override
    public void initialize(){
        algaeHandlerSubsystem.intake();
    }

    @Override
    public void end(boolean interupt){
        algaeHandlerSubsystem.stopIntake();
    }
}
