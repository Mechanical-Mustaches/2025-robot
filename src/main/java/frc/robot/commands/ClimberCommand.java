package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

public class ClimberCommand extends Command {
    ClimberSubsystem climberSubsystem;
    ClimberSubsystem.Stage stage;
    SuperstructureSubsystem superstructureSubsystem;

    public ClimberCommand(ClimberSubsystem climberSubsystem, ClimberSubsystem.Stage stage,
            SuperstructureSubsystem superstructureSubsystem) {

        this.climberSubsystem = climberSubsystem;
        this.stage = stage;
        this.superstructureSubsystem = superstructureSubsystem;
    }

    @Override
    public void initialize() {
        climberSubsystem.climb();
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.isDone(stage);

    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.climberStop();
    }
}
