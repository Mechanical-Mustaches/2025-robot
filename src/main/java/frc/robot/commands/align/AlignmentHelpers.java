package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem.ReefPosition;

public class AlignmentHelpers {
    private PIDController pidRotation = new PIDController(0.15, 0.01, 0);

    public AlignmentHelpers() {
        SmartDashboard.putBoolean("align/rotation/isRotated", false);
        SmartDashboard.putNumber("align/rotation/rotationalVelocity", 0);
        SmartDashboard.putNumber("align/rotation/currentAngle", 0);
        SmartDashboard.putNumber("align/rotation/desiredNegativeAngle", 0);
        SmartDashboard.putNumber("align/rotation/desiredPositiveAngle", 0);
    }

    public void initialize() {
        pidRotation.reset();
    }

    public boolean isRotated(ReefPosition reefPosition, Pose2d robotPose) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotPose.getRotation().getDegrees();

        boolean result = Math.abs(desiredNegativeAngle - currentAngle) < Constants.ROTATION_ALIGNMENT_TOLERANCE 
                || Math.abs(desiredPositiveAngle - currentAngle) < Constants.ROTATION_ALIGNMENT_TOLERANCE;

        SmartDashboard.putBoolean("align/rotation/isRotated", result);
        return result;
    }

    public double getRotation(ReefPosition reefPosition, Pose2d robotPose) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotPose.getRotation().getDegrees();

        SmartDashboard.putNumber("align/rotation/currentAngle", currentAngle);
        SmartDashboard.putNumber("align/rotation/desiredNegativeAngle", desiredNegativeAngle);
        SmartDashboard.putNumber("align/rotation/desiredPositiveAngle", desiredPositiveAngle);

        double rotation;
        if (Math.abs(desiredPositiveAngle - currentAngle) > Math.abs(desiredNegativeAngle - currentAngle)) {
            rotation = pidRotation.calculate(currentAngle, desiredNegativeAngle);
        } else {
            rotation = pidRotation.calculate(currentAngle, desiredPositiveAngle);
        }

        SmartDashboard.putNumber("align/rotation/rotationalVelocity", currentAngle);
        return rotation;
    }
}
