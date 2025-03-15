package frc.robot.commands.align;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveToWallCommand extends Command {
    private final SwerveDriveSubsystem swerve;

    public DriveToWallCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {
        // TODO
        swerve.driveRobotRelative(new ChassisSpeeds(0.3, 0, 0));
    }
}
