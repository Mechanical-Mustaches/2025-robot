package frc.robot.commands.align;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.commands.align.Constants.Mode;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PreciseAlignCommand extends Command {
    private SwerveDriveSubsystem swerve;
    private AlignmentHelpers alignmentHelpers = new AlignmentHelpers();
    private PIDController tagPidController = new PIDController(4, 0.3, 0);
    private PIDController rotationPidController = new PIDController(.058, 0, 0);

    private SwerveDriveSubsystem.ReefPosition closestReef;
    private Constants.Mode mode;

    public PreciseAlignCommand(SwerveDriveSubsystem swerve, Constants.Mode mode) {
        this.swerve = swerve;
        this.mode = mode;
        addRequirements(swerve);

        SmartDashboard.putNumber("align/zPose", 0);
        SmartDashboard.putNumber("align/xPose", 0);
        SmartDashboard.putNumber("align/tag-rotation/x", 0);
        SmartDashboard.putNumber("align/tag-rotation/y", 0);
        SmartDashboard.putNumber("align/tag-rotation/z", 0);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("align/state", "PRECISE");
        alignmentHelpers.initialize();
        tagPidController.reset();
        closestReef = swerve.getClosestReefPosition();

    }

    private Optional<Pose3d> getTagPose() {
        String llName = this.mode == Constants.Mode.LEFT ? "limelight-right" : "limelight-left";

        if (LimelightHelpers.getTV(llName)) {
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
            return Optional.of(targetPose);
        }

        return Optional.empty();
    }

    private Optional<Pose3d> getBotPoseTagSpace() {
        String llName = this.mode == Constants.Mode.LEFT ? "limelight-right" : "limelight-left";

        if (LimelightHelpers.getTV(llName)) {
            Pose3d botPose = LimelightHelpers.getBotPose3d_TargetSpace(llName);
            return Optional.of(botPose);
        }

        return Optional.empty();
    }

    private boolean isRotated() {
        if (Constants.USE_TAG_SPACE) {
            return alignmentHelpers.isRotated(closestReef, swerve.getPose().getRotation(), 6);
        }
        return alignmentHelpers.isRotated(closestReef, swerve.getPose().getRotation());
    }

    private double getXSetpoint() {
        if (mode == Mode.LEFT) {
            return Constants.REEF_POLE_CENTER_OFFSET;
        } else if (mode == Mode.RIGHT) {
            return -Constants.REEF_POLE_CENTER_OFFSET;
        }

        throw new Error(String.format("Unknown mode: %s", mode));
    }

    @Override
    public void execute() {
        double vy = 0;
        double vx = 0;
        double rotation = alignmentHelpers.getRotation(closestReef, swerve.getPose().getRotation());

        if (Constants.USE_TAG_SPACE) {
            Optional<Pose3d> botPose = getBotPoseTagSpace();
            if (botPose.isPresent() && isRotated()) {
                Pose3d pose = botPose.get();
                vy = tagPidController.calculate(pose.getX(), -getXSetpoint()); // Left right is flipped for tag space
                vx = -tagPidController.calculate(pose.getZ(), Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT) / 2;
                rotation = -rotationPidController.calculate(pose.getRotation().getY(), 0);
            }
        } else {
            Optional<Pose3d> targetPose = getTagPose();
            if (targetPose.isPresent() && isRotated()) {
                Pose3d pose = targetPose.get();
                vy = tagPidController.calculate(pose.getX(), getXSetpoint());
                vx = -tagPidController.calculate(pose.getZ(), Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT) / 2;

                Rotation3d poseRotation = pose.getRotation();

                SmartDashboard.putNumber("align/zPose", pose.getZ());
                SmartDashboard.putNumber("align/xPose", pose.getX());
                SmartDashboard.putNumber("align/tag-rotation/x", poseRotation.getX());
                SmartDashboard.putNumber("align/tag-rotation/y", poseRotation.getY());
                SmartDashboard.putNumber("align/tag-rotation/z", poseRotation.getZ());
            }
        }

        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, rotation));
    }

    private boolean inTolerance(double ref, double setPoint) {
        return Math.abs(ref - setPoint) < Constants.PRECISE_ALIGNMENT_POSITION_TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        if (!isRotated()) {
            return false;
        }

        if (Constants.USE_TAG_SPACE) {
            Optional<Pose3d> botPose = getBotPoseTagSpace();
            if (botPose.isEmpty()) {
                return true;
            }

            Pose3d pose = botPose.get();
            if (inTolerance(pose.getX(), -getXSetpoint())
                    && inTolerance(pose.getZ(), Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT)) {
                return true;
            }
        } else {
            Optional<Pose3d> targetPose = getTagPose();
            if (targetPose.isEmpty()) {
                return true;
            }

            Pose3d pose = targetPose.get();
            if (inTolerance(pose.getX(), getXSetpoint())
                    && inTolerance(pose.getZ(), Constants.PRECISE_ALIGNMENT_FORWARD_SETPOINT)) {
                return true;
            }
        }

        return false;

    }
}
