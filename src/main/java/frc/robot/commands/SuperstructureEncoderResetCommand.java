package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class SuperstructureEncoderResetCommand extends Command{
    SuperstructureSubsystem superstructureSubsystem;

    public SuperstructureEncoderResetCommand(SuperstructureSubsystem subsystem){
        this.superstructureSubsystem = subsystem;
    }

    @Override
    public void initialize(){
        superstructureSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
