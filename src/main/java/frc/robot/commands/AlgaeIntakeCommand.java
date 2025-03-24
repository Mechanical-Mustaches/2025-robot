package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeHandlerSubsystem algaeHandlerSubsystem;

    public AlgaeIntakeCommand(AlgaeHandlerSubsystem algaeHandlerSubsystem) {
        this.algaeHandlerSubsystem = algaeHandlerSubsystem;
        this.addRequirements(algaeHandlerSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("algae_intake/state", "intake");
    }

    @Override
    public void execute() {
        algaeHandlerSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        if (algaeHandlerSubsystem.isAlgaeDetected()) {
            algaeHandlerSubsystem.hold();
        } else {
            algaeHandlerSubsystem.stopIntake();
        }

    }

    @Override
    public boolean isFinished() {
        return algaeHandlerSubsystem.isAlgaeDetected();
    }

}
