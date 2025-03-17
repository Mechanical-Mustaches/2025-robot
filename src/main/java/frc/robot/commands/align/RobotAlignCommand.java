package frc.robot.commands.align;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignCommand extends SequentialCommandGroup {
    public RobotAlignCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        super(
            new RoughAlignCommand(swerveDriveSubsystem),
            new PreciseAlignCommand(swerveDriveSubsystem),
            new DriveToWallCommand(swerveDriveSubsystem)
        );
    }
}
