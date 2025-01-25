package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTelemetry extends Command{
    private final ElevatorSubsystem elevator;

    public ElevatorTelemetry(ElevatorSubsystem subsystem){
        elevator = subsystem;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber( "ElevatorEncoderValue", elevator.getEncoderValue());
    }

   @Override
   public boolean isFinished() {
       return false;
   }

}
