package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RoughAlignCommand extends Command {
    // Distance from center point of reef to end command and switch to precise
    // alignment
    public static final double FINISH_DISTANCE = 0.5;
    private final double ROUGH_ALIGN_OFFSET = 0.6;

    private SwerveDriveSubsystem swerve;
    private SwerveDriveSubsystem.ReefPosition closestReef;

    private Command driveCommand;

    public RoughAlignCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("align/state", "ROUGH");
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

        // double distanceFromCenterOfReef = swerve.getPose().getTranslation().getDistance(closestReef.translation());
        // boolean inRange = Math.abs(distanceFromCenterOfReef - ROUGH_ALIGN_OFFSET) < 0.1;
        // return inRange || driveCommand.isFinished();
        return driveCommand.isFinished();
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
