package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class AlgaePivotTelemetry extends Command{
    private final AlgaeHandlerSubsystem pivot;

    public AlgaePivotTelemetry(AlgaeHandlerSubsystem pivot){
        this.pivot = pivot;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber( "PivotEncoderValue", pivot.getEncoderValue());
    }

   @Override
   public boolean isFinished() {
       return false;
   }

}
