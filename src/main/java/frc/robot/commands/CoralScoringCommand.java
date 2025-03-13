package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralScoringCommand extends Command {
    EndEffectorSubsystem endEffector;
    ElevatorSubsystem elevatorSubsystem;

    public CoralScoringCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevatorSubsystem) {
        this.endEffector = endEffector;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        if (elevatorSubsystem.getEncoderValue() <= ElevatorSubsystem.Level.L1.encoderValue + 2) {
            endEffector.L1Score();
        } else if (elevatorSubsystem.getEncoderValue() > ElevatorSubsystem.Level.L3.encoderValue + 2) {
            endEffector.L4Score();
        } else {
            endEffector.effectorScore();
        }

    }

    @Override
    public void end(boolean interupt) {
        endEffector.effectorStop();
    }

    @Override
    public boolean isFinished() {
        return !endEffector.isCoralSeenFront();
    }
}
