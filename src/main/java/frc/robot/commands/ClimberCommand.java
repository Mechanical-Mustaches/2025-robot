package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    ClimberSubsystem climberSubsystem;
    ClimberSubsystem.Stage stage;

    public ClimberCommand(ClimberSubsystem climberSubsystem, ClimberSubsystem.Stage stage){

        this.climberSubsystem = climberSubsystem;
        this.stage = stage;


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
