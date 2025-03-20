package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.AlgaeHandlerSubsystem.Position;

public class AlgaeIntakeCommandGroup extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    SequentialCommandGroup commandGroup;

    public AlgaeIntakeCommandGroup(AlgaeHandlerSubsystem algaeHandlerSubsystem) {
        this.algaeHandlerSubsystem = algaeHandlerSubsystem;
        this.commandGroup = new SequentialCommandGroup(
                new DumbAlgaePivotCommand(algaeHandlerSubsystem, Position.Out),
                new AlgaeIntakeCommand(algaeHandlerSubsystem));
        // new DumbAlgaePivotCommand(algaeHandlerSubsystem, Position.In));
    }

    @Override
    public void initialize() {
        this.commandGroup.schedule();
        algaeHandlerSubsystem.lock();
    }

    @Override
    public void end(boolean interrupted) {
        algaeHandlerSubsystem.unlock();
    }
}
