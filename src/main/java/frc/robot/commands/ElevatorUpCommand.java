package frc.robot.commands;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorUpCommand extends Command{
    private ElevatorSubsystem eleSubsystem;

    public ElevatorCommand(ElevatorSubsystem subsystem) {
        eleSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
      }
       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ElevatorSubsystem.raiseElevator(1)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//motor power 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check encoder val
    return false;
  }
}
