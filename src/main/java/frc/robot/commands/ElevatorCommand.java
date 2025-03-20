package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorSubsystem.Level targetLevel;
    private final EndEffectorSubsystem endEffector;

    public ElevatorCommand(ElevatorSubsystem subsystem, ElevatorSubsystem.Level level, EndEffectorSubsystem endEffector,
            boolean isIntakingAlgae) {
        elevator = subsystem;
        targetLevel = level;
        this.endEffector = endEffector;
    }

    @Override
    public void initialize() {
        // if (endEffector.isCoralSeenBack()) {
        // return;
        // }

        elevator.setPosition(targetLevel, endEffector);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
