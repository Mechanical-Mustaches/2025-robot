package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

public class DumbAlgaeIntakeCommand extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    public DumbAlgaeIntakeCommand(AlgaeHandlerSubsystem algaeSubsystem, ElevatorSubsystem eleSubsystem) {
        algaeHandlerSubsystem = algaeSubsystem;
        elevatorSubsystem = eleSubsystem;

    }

    @Override
    public void initialize() {
        if (ElevatorSubsystem.Level.L1.encoderValue + 5 < elevatorSubsystem.getEncoderValue()
                && ElevatorSubsystem.Level.L2.encoderValue + 5 > elevatorSubsystem.getEncoderValue()) {
            elevatorSubsystem.setPosition(Level.LAlgaeBottom, null);
        } else if (ElevatorSubsystem.Level.L2.encoderValue + 5 <= elevatorSubsystem.getEncoderValue()) {
            elevatorSubsystem.setPosition(Level.LAlgaeTop, null);
        } else {
            algaeHandlerSubsystem.intake();
        }

    }

    @Override
    public void execute() {
        algaeHandlerSubsystem.intake();
    }

    @Override
    public void end(boolean interupt) {
        algaeHandlerSubsystem.launch();
    }
}
