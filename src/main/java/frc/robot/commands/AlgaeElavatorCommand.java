package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeElavatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    public AlgaeElavatorCommand(ElevatorSubsystem subsystem) {
        elevator = subsystem;
    }

    @Override
    public void initialize() {
        elevator.setPosition(elevator.getNearestAlgaeLevel());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
