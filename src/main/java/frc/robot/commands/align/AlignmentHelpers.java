package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem.ReefPosition;

public class AlignmentHelpers {
    private PIDController pidRotation = new PIDController(0.15, 0.02, 0);

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

    public boolean isRotated(ReefPosition reefPosition, Rotation2d robotHeading) {
        return this.isRotated(reefPosition, robotHeading, Constants.ROTATION_ALIGNMENT_TOLERANCE);
    }

    public boolean isRotated(ReefPosition reefPosition, Rotation2d robotHeading, double tolerance) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotHeading.getDegrees();

        boolean result = Math.abs(desiredNegativeAngle - currentAngle) < tolerance
                || Math.abs(desiredPositiveAngle - currentAngle) < tolerance;

        SmartDashboard.putBoolean("align/rotation/isRotated", result);
        return result;
    }

    public double getRotation(ReefPosition reefPosition, Rotation2d robotHeading) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotHeading.getDegrees();

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
