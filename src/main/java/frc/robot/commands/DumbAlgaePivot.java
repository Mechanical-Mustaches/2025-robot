package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;

public class DumbAlgaePivot extends Command{
    AlgaeHandlerSubsystem algaeHandlerSubsystem;
    double motorSpeed;

    public DumbAlgaePivot(AlgaeHandlerSubsystem subsystem,double speed){
        algaeHandlerSubsystem = subsystem;
    }

    @Override
    public void initialize(){
        algaeHandlerSubsystem.dumbPivot(motorSpeed);
    }

    @Override
    public void end(boolean interupt){
        algaeHandlerSubsystem.stopPivot();
    }
}
