package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class AlgaePivotCommand extends Command{
    AlgaeHandlerSubsystem pivot;
    AlgaeHandlerSubsystem intakeActivator;

    public AlgaePivotCommand(AlgaeHandlerSubsystem pivot, AlgaeHandlerSubsystem intakeActivator){
    this.pivot = pivot;
    this.intakeActivator = intakeActivator;
    }
    @Override
    public void initialize(){
        intakeActivator.intake();
        pivot.verticalPivot();
    }

    @Override
    public void execute(){
        pivot.pivot();
    }

    @Override
    public void end(boolean interupt){
        intakeActivator.stopIntake();
        pivot.verticalPivot();
    }
    
}
