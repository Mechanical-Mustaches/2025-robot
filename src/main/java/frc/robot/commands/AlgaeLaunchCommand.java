package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class AlgaeLaunchCommand extends Command {
    private final AlgaeHandlerSubsystem subsystem;

    public AlgaeLaunchCommand(AlgaeHandlerSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        this.subsystem.launch();
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.stopIntake();
    }
}
