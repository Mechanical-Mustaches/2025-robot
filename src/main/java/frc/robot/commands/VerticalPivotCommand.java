package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class VerticalPivotCommand extends Command{
    AlgaeHandlerSubsystem pivot;

    public VerticalPivotCommand(AlgaeHandlerSubsystem pivot){
    this.pivot = pivot;
    }
    @Override
    public void initialize(){
        pivot.verticalPivot();
    }
   
    @Override
    public void end(boolean interupt){
       pivot.stopPivot();
    }
}
