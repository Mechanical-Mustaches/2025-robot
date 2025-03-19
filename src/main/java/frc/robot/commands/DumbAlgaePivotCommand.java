package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaePivotCommand extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    AlgaeHandlerSubsystem.Position targetPosition;

    public DumbAlgaePivotCommand(AlgaeHandlerSubsystem subsystem, AlgaeHandlerSubsystem.Position targetPosition) {
        algaeHandlerSubsystem = subsystem;
        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        algaeHandlerSubsystem.pivot(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
