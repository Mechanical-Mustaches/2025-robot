package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorSubsystem.Level targetLevel;

    public ElevatorCommand(ElevatorSubsystem subsystem, ElevatorSubsystem.Level level) {
        elevator = subsystem;
        targetLevel = level;
    }

    @Override
    public void initialize() {
        // if (endEffector.isCoralSeenBack()) {
        // return;
        // }

        elevator.setPosition(targetLevel);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
