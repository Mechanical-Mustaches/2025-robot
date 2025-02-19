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
        if (intakeActivator.isAlgaeDetected() > 5){
            pivot.horizontalPivot();
            intakeActivator.stopIntake();
        }
    }

    @Override
    public void end(boolean interupt){
        intakeActivator.stopIntake();
        pivot.verticalPivot();
    }
    
}
