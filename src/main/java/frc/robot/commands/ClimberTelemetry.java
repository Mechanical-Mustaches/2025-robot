package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTelemetry extends Command {
    private final ClimberSubsystem climber;

    public ClimberTelemetry(ClimberSubsystem climber){
        this.climber = climber;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber( "ClimberEncoderValue", climber.getEncoderValue());
    }

   @Override
   public boolean isFinished() {
       return false;
   }

}
