package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class KeepClosedCommand extends Command {
    SuperstructureSubsystem superstructure;
    public KeepClosedCommand(SuperstructureSubsystem superstructure){
        this.superstructure = superstructure;
    }
    @Override
    public void initialize(){
        superstructure.keepClosed();
    }
    @Override
    public void end(boolean interupt){
       superstructure.keepClosedStop();
    }
}
