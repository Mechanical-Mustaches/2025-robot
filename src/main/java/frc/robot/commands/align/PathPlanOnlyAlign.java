package frc.robot.commands.align;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.align.Constants.Mode;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PathPlanOnlyAlign extends DeferredCommand {
    public PathPlanOnlyAlign(SwerveDriveSubsystem swerve, Constants.Mode mode) {
        super(() -> getDriveCommand(swerve, mode), Set.of(swerve));
    }

    private static Command getDriveCommand(SwerveDriveSubsystem swerve, Constants.Mode mode) {
        SwerveDriveSubsystem.ReefPosition closestReef = swerve.getClosestReefPosition();
        double reefAngle = closestReef.rotation().getRadians() - Math.PI;
        double reefAngle2 = 0;
        if (mode == Mode.LEFT){
            reefAngle2 = reefAngle - Math.PI/2;
        } else if (mode == Mode.RIGHT){
            reefAngle2  = reefAngle + Math.PI/2;
        }

        // Normal offset from center of reef
        double desiredPositionX = closestReef.translation().getX()
                + Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT * Math.cos(reefAngle);
        double desiredPositionY = closestReef.translation().getY()
                + Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT * Math.sin(reefAngle);

        desiredPositionX += Constants.REEF_POLE_CENTER_OFFSET * Math.cos(reefAngle2);
        desiredPositionY += Constants.REEF_POLE_CENTER_OFFSET * Math.sin(reefAngle2);
        
        Pose2d desiredPose = new Pose2d(desiredPositionX, desiredPositionY, closestReef.rotation());
        return swerve.goToWaypoint(desiredPose, 2, 2, Math.PI, 2 * Math.PI, 0);
    }
}
