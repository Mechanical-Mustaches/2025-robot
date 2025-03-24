package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaePivotCommand extends Command {
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    AlgaeHandlerSubsystem.Position targetPosition;

    public DumbAlgaePivotCommand(AlgaeHandlerSubsystem subsystem, AlgaeHandlerSubsystem.Position targetPosition) {
        algaeHandlerSubsystem = subsystem;
        this.targetPosition = targetPosition;
        this.addRequirements(algaeHandlerSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("algae_intake/state", "pivot " + targetPosition.toString());
        algaeHandlerSubsystem.pivot(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
