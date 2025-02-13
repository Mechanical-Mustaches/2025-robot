package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem.Stage;

public class SuperstructureDefaultCommand extends Command{
    SuperstructureSubsystem superstructureSubsystem;

    public SuperstructureDefaultCommand(SuperstructureSubsystem superstructureSubsystem){
        this.superstructureSubsystem = superstructureSubsystem;
    }

    @Override
    public void initialize() {
        superstructureSubsystem.toStage(Stage.Closed);
    }

    @Override
     public boolean isFinished() {
       return true;
     }

}
