package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.AlgaeHandlerSubsystem.Position;

public class AlgaeIntakeCommandGroup extends SequentialCommandGroup {

    public AlgaeIntakeCommandGroup(AlgaeHandlerSubsystem algaeHandlerSubsystem) {
        super(
                new DumbAlgaePivotCommand(algaeHandlerSubsystem, Position.Out),
                new AlgaeIntakeCommand(algaeHandlerSubsystem),
                new DumbAlgaePivotCommand(algaeHandlerSubsystem, Position.In));
    }
}
