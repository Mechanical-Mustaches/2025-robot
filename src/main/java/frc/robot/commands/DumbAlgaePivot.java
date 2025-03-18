package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaePivot extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    double motorSpeed;

    public DumbAlgaePivot(AlgaeHandlerSubsystem subsystem, double speed) {
        algaeHandlerSubsystem = subsystem;
        motorSpeed = speed;
    }

    @Override
    public void initialize() {
        if (motorSpeed > 0) {
            algaeHandlerSubsystem.pivotOut(motorSpeed);
        } else if (0 > motorSpeed) {
            algaeHandlerSubsystem.pivotIn(motorSpeed);
        } else {
            algaeHandlerSubsystem.stopPivot();
        }
    }

    @Override
    public void execute() {
        if (motorSpeed > 0) {
            algaeHandlerSubsystem.pivotOut(motorSpeed);
        } else if (0 > motorSpeed) {
            algaeHandlerSubsystem.pivotIn(motorSpeed);
        } else {
            algaeHandlerSubsystem.stopPivot();
        }
    }

    @Override
    public void end(boolean interupt) {
        algaeHandlerSubsystem.stopPivot();
    }
}
