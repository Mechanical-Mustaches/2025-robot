package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignV2Command extends Command {

    public enum Mode {
        left,
        right,
        manual
    }

    SwerveDriveSubsystem swerve;
    private boolean autoFinish;

    private PIDController pidRotation = new PIDController(0.1, 0.005, 0);
    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(5, 0.01, 0);
    private PIDController roughPidController = new PIDController(1, 0, 0);

    private static final double reefPoleCenterOffset = 0.1558544;

    private DoubleSupplier horizontalInput;
    private final double rotationTolerance = 15;
    private final double distanceTolerance = 10;
    private final double tagTolerance = 0.6;
    private final double defaultVelocity;

    private final double wallDistanceSetpoint = 115;

    SwerveDriveSubsystem.ReefPosition closestReef;

    private final double roughAlignOffset = 0.4;
    private final double roughAlignOffsetX = 0.165;
    private final double roughAlignOffsetY = 0;
    private boolean testRoughAlign = false;
    private boolean roughValidity;

    public RobotAlignV2Command(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
        // this.mode = mode;
        this.autoFinish = false;
        defaultVelocity = swerve.getMaximumChassisVelocity() / 10;
    }

    @Override
    public void initialize() {
        pidRotation.reset();
        wallPidController.reset();
        tagPidController.reset();
        closestReef = swerve.getClosestReefPosition();
        roughValidity = true;

    }

    private double getDistanceToWall() {
        return swerve.rightDistanceSensor.getRange();
    }

    @Override
    public void execute() {
        double distanceFromCenterOfReef = swerve.getPose().getTranslation().getDistance(closestReef.translation());
        boolean distanceValidity = swerve.rightDistanceSensor.getRange() > 100
                && swerve.rightDistanceSensor.isRangeValid();

        if (distanceFromCenterOfReef < 0.5 && distanceValidity && !testRoughAlign) {
            preciseAlign();
            roughValidity = false;
        } else if (roughValidity) {
            roughAlign();
        }

    }

    private double getRotation() {
        double desiredPositiveAngle = closestReef.rotation().getDegrees();
        double desiredNegativeAngle = closestReef.rotation().getDegrees() - 360;
        double currentAngle = swerve.getPose().getRotation().getDegrees();
        SmartDashboard.putNumber("rav/currentRotation", currentAngle);
        SmartDashboard.putNumber("rav/desiredNegativeRotation", desiredNegativeAngle);
        SmartDashboard.putNumber("rav/desiredPositiveRotation", desiredPositiveAngle);
        if (Math.abs(desiredPositiveAngle - currentAngle) > Math.abs(desiredNegativeAngle - currentAngle)) {
            return pidRotation.calculate(currentAngle, desiredNegativeAngle);
        } else {
            return pidRotation.calculate(currentAngle, desiredPositiveAngle);
        }
    }

    private void preciseAlign() {
        double vy = 0;
        double vx = 0;
        if (LimelightHelpers.getTV("limelight-right")) {
            Pose3d leftTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-right");
            vy = tagPidController.calculate(leftTargetPose.getX(), reefPoleCenterOffset);
            SmartDashboard.putNumber("rav/TargetX", leftTargetPose.getX());
        }

        double distanceToWall = getDistanceToWall();

        if (Math.abs(wallDistanceSetpoint - distanceToWall) > distanceTolerance) {
            vx = -wallPidController.calculate(distanceToWall, wallDistanceSetpoint);
        }

        SmartDashboard.putNumber("rav/velocityY", vy);

        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, getRotation()));
    }

    private void roughAlign() {
        // double vy = 0;
        // double vx = 0;
        // double reefAngle = closestReef.rotation().getRadians() - Math.PI;
        // double currentPositionX = swerve.getPose().getX();
        // double currentPositionY = swerve.getPose().getY();
        // double leftDesiredPositionX = closestReef.translation().getX() +
        // roughAlignOffset * Math.cos(reefAngle)
        // + roughAlignOffset * Math.sin(reefAngle - 90);
        // double desiredPositionY = closestReef.translation().getY() + roughAlignOffset
        // * Math.sin(reefAngle)
        // double deltaX = desiredPositionX - currentPositionX;
        // double deltaY = desiredPositionY - currentPositionY;
        // double m = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        // vx = deltaX/m;
        // vy = deltaY/m;

        // SmartDashboard.putNumber("align/rough/currentPositionX", currentPositionX);
        // SmartDashboard.putNumber("align/rough/currentPositionY", currentPositionY);
        // SmartDashboard.putNumber("align/rough/desiredPositionX", desiredPositionX);
        // SmartDashboard.putNumber("align/rough/desiredPositionY", desiredPositionY);

        // swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, getRotation()));
    }

    // @Override
    // public boolean isFinished(){
    // if (!autoFinish){
    // return false;
    // }
    // boolean isRotated = (Math.abs(getDistanceDifference()) < rotationTolerance);
    // boolean isVerticallyAligned = Math.abs(wallDistanceSetpoint -
    // getDistanceToWall()) < distanceTolerance;
    // boolean isHorizontallyAlignedLeft = this.mode == Mode.left &&
    // LimelightHelpers.getTV("limelight-right") &&
    // Math.abs(LimelightHelpers.getTX("limelight-right") - leftModeTagSetpoint) <
    // tagTolerance;

    // boolean isHorizontallyAlignedRight = this.mode == Mode.right &&
    // LimelightHelpers.getTV("limelight-left") &&
    // Math.abs(LimelightHelpers.getTX("limelight-left") - rightModeTagSetpoint) <
    // tagTolerance;

    // return isRotated && isVerticallyAligned && (isHorizontallyAlignedLeft ||
    // isHorizontallyAlignedRight);

    // }

}
