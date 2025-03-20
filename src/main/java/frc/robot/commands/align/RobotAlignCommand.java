package frc.robot.commands.align;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignCommand extends SequentialCommandGroup {
    public RobotAlignCommand(SwerveDriveSubsystem swerveDriveSubsystem, Constants.Mode mode, boolean isInAuto) {
        if (isInAuto){
            addCommands(
            new PreciseAlignCommand(swerveDriveSubsystem, mode),
            new ParallelDeadlineGroup(
                new WaitCommand(.5),
                new DriveToWallCommand(swerveDriveSubsystem)
            )
        );
        } else {
            addCommands(
            new RoughAlignCommand(swerveDriveSubsystem, mode),
            new PreciseAlignCommand(swerveDriveSubsystem, mode),
            new DriveToWallCommand(swerveDriveSubsystem)
        );
        }
    }

    public RobotAlignCommand(SwerveDriveSubsystem swerveDriveSubsystem, Constants.Mode mode) {
        this(swerveDriveSubsystem, mode, false);
    }

    
}
