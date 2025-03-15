package frc.robot.commands.align;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PreciseAlignCommand extends Command {
    private SwerveDriveSubsystem swerve;
    private AlignmentHelpers alignmentHelpers;

    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(5, 0.01, 0);

    private static final double reefPoleCenterOffset = 0.1558544;

    private final double distanceTolerance = 10;
    private final double wallDistanceSetpoint = 115;

    SwerveDriveSubsystem.ReefPosition closestReef;

    public PreciseAlignCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        alignmentHelpers.initialize();
        wallPidController.reset();
        tagPidController.reset();
        closestReef = swerve.getClosestReefPosition();

    }

    private double getDistanceToWall() {
        return swerve.rightDistanceSensor.getRange();
    }

    private Optional<Pose3d> getTagPose(boolean rightCamera) {
        String llName = rightCamera ? "limelight-right" : "limelight-left";

        if (LimelightHelpers.getTV(llName)){
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
            return Optional.of(targetPose);
        }

        return Optional.empty();
    }

    @Override
    public void execute() {
        double vy = 0;
        double vx = 0;
        double rotation = alignmentHelpers.getRotation(closestReef, swerve.getPose());

        Optional<Pose3d> targetPose = getTagPose(true);
        if (targetPose.isPresent()) {
            Pose3d pose = targetPose.get();
            vy = tagPidController.calculate(pose.getX(), reefPoleCenterOffset);
            SmartDashboard.putNumber("pac/TargetX", pose.getX());
        }

        // TODO: Use limelight distance instead
        double distanceToWall = getDistanceToWall();
        if (Math.abs(wallDistanceSetpoint - distanceToWall) > distanceTolerance) {
            vx = -wallPidController.calculate(distanceToWall, wallDistanceSetpoint);
        }

        SmartDashboard.putNumber("pac/velocityY", vy);

        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, rotation));
    }

    @Override
    public boolean isFinished() {
        return getTagPose(true).isEmpty();
    }
}
