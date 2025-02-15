package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class HorizontalPivotCommand extends Command{
    AlgaeHandlerSubsystem pivot;

    public HorizontalPivotCommand(AlgaeHandlerSubsystem pivot){
    this.pivot = pivot;
    }
    @Override
    public void initialize(){
        pivot.horizontalPivot();
    }
   
    @Override
    public void end(boolean interupt){
       pivot.stopPivot();
    }
}
