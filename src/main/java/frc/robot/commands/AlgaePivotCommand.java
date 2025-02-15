package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class AlgaePivotCommand extends Command{
    AlgaeHandlerSubsystem pivot;
    Boolean horizontalPivot;

    public AlgaePivotCommand(AlgaeHandlerSubsystem pivot, boolean horizontalPivot){
    this.pivot = pivot;
    this.horizontalPivot = horizontalPivot;
    }
    @Override
    public void initialize(){
        if(horizontalPivot){
            pivot.horizontalPivot();
        } else{
            pivot.verticalPivot();
        }
        
    }
   
    // @Override
    // public void end(boolean interupt){
    //    pivot.stopPivot();
    // }

    @Override
    public boolean isFinished(){
        return true;
    }
}
