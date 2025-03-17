package frc.robot.commands.align;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.ReefPosition;

public class DriveToWallCommand extends Command {
    private final SwerveDriveSubsystem swerve;
    private final AlignmentHelpers alignmentHelpers = new AlignmentHelpers();
    private ReefPosition closestReef;

    public DriveToWallCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("align/state", "DRIVE_TO_WALL");
        closestReef = swerve.getClosestReefPosition();
    }

    @Override
    public void execute() {
        double rotation = alignmentHelpers.getRotation(closestReef, swerve.getPose());
        swerve.driveRobotRelative(new ChassisSpeeds(.8, 0, rotation));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("align/state", "INACTIVE");
    }
}
