package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeElavatorCommand extends Command {
    private final ElevatorSubsystem elevator;

    public AlgaeElavatorCommand(ElevatorSubsystem subsystem) {
        elevator = subsystem;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("algae_intake/state", "elevator");
        elevator.setPosition(elevator.getNearestAlgaeLevel());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
