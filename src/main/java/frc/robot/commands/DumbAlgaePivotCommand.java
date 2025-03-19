package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaePivotCommand extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    boolean pivotOut;

    public DumbAlgaePivotCommand(AlgaeHandlerSubsystem subsystem, boolean pivotOut) {
        algaeHandlerSubsystem = subsystem;
        this.pivotOut = pivotOut;
    }

    @Override
    public void initialize() {
        if (pivotOut) {
            algaeHandlerSubsystem.pivotOut();
        } else{
            algaeHandlerSubsystem.pivotIn();
        }
    }

    @Override
    public void execute() {
        if (pivotOut) {
            algaeHandlerSubsystem.pivotOut();
        } else {
            algaeHandlerSubsystem.pivotIn();
        }
    }

    @Override
    public void end(boolean interupt) {
        algaeHandlerSubsystem.stopPivot();
    }
}
