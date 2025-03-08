package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SuperstructureSubsystem;

public class OpenDoorCommand extends Command{
    private SequentialCommandGroup commandGroup;

    public OpenDoorCommand(SuperstructureSubsystem superstructureSubsystem){
        this.commandGroup = new SequentialCommandGroup(
            new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), -0.2),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getRightMotor(), -0.2),
        new WaitCommand(0.5),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), 0.2),
        new WaitCommand(1.5),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), 0),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getRightMotor(), 0));

    }
    @Override 
    public void initialize(){
        if (DriverStation.isTeleop() && DriverStation.getMatchTime() < 35) {
            CommandScheduler.getInstance().schedule(this.commandGroup);
        } 
    }
}
