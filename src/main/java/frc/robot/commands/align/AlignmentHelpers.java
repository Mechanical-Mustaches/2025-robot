package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem.ReefPosition;

public class AlignmentHelpers {
    private PIDController pidRotation = new PIDController(0.15, 0.005, 0);

    public void initialize() {
        pidRotation.reset();
    }

    public boolean isRotated(ReefPosition reefPosition, Pose2d robotPose) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotPose.getRotation().getDegrees();

        return Math.abs(desiredNegativeAngle - currentAngle) < Constants.ROTATION_ALIGNMENT_TOLERANCE 
                || Math.abs(desiredPositiveAngle - currentAngle) < Constants.ROTATION_ALIGNMENT_TOLERANCE;
    }

    public double getRotation(ReefPosition reefPosition, Pose2d robotPose) {
        double desiredPositiveAngle = reefPosition.rotation().getDegrees();
        double desiredNegativeAngle = reefPosition.rotation().getDegrees() - 360;
        double currentAngle = robotPose.getRotation().getDegrees();

        SmartDashboard.putNumber("alignment/currentRotation", currentAngle);
        SmartDashboard.putNumber("alignment/desiredNegativeRotation", desiredNegativeAngle);
        SmartDashboard.putNumber("alignment/desiredPositiveRotation", desiredPositiveAngle);

        if (Math.abs(desiredPositiveAngle - currentAngle) > Math.abs(desiredNegativeAngle - currentAngle)) {
            return pidRotation.calculate(currentAngle, desiredNegativeAngle);
        } else {
            return pidRotation.calculate(currentAngle, desiredPositiveAngle);
        }
    }
}
