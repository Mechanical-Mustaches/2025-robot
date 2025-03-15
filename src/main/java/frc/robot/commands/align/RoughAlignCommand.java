package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RoughAlignCommand extends Command {
    // Distance from center point of reef to end command and switch to precise
    // alignment
    public static final double FINISH_DISTANCE = 0.5;
    private final double ROUGH_ALIGN_OFFSET = 0.45;

    private SwerveDriveSubsystem swerve;
    private SwerveDriveSubsystem.ReefPosition closestReef;

    private Command driveCommand;

    public RoughAlignCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        closestReef = swerve.getClosestReefPosition();
        double reefAngle = closestReef.rotation().getRadians() - Math.PI;
        double desiredPositionX = closestReef.translation().getX() + ROUGH_ALIGN_OFFSET * Math.cos(reefAngle);
        double desiredPositionY = closestReef.translation().getY() + ROUGH_ALIGN_OFFSET * Math.sin(reefAngle);

        Pose2d desiredPose = new Pose2d(desiredPositionX, desiredPositionY, closestReef.rotation());
        driveCommand = swerve.goToWaypoint(desiredPose);
        driveCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        if (driveCommand == null) {
            return true;
        }

        double distanceFromCenterOfReef = swerve.getPose().getTranslation().getDistance(closestReef.translation());
        boolean inRange = distanceFromCenterOfReef < 0.55 && distanceFromCenterOfReef > 0.4;
        return inRange || driveCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (driveCommand == null) {
            return;
        }

        driveCommand.cancel();
        driveCommand = null;
    }
}
