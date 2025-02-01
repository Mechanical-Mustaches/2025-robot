package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;


public class AlgaeTelemetry extends Command{
    private final AlgaeHandlerSubsystem AlgaeSubsystem;

    public AlgaeTelemetry(AlgaeHandlerSubsystem subsystem){
        AlgaeSubsystem = subsystem;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber( "AlgaePivotEncoderValue", AlgaeSubsystem.getEncoderValue());
    }

   @Override
   public boolean isFinished() {
       return false;
   }

}
