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
    private AlignmentHelpers alignmentHelpers = new AlignmentHelpers();

    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(10, 0.03, 0);

    private static final double reefPoleCenterOffset = 0.1558544;

    SwerveDriveSubsystem.ReefPosition closestReef;

    public PreciseAlignCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("align/state", "PRECISE");
        alignmentHelpers.initialize();
        wallPidController.reset();
        tagPidController.reset();
        closestReef = swerve.getClosestReefPosition();

    }

    private Optional<Pose3d> getTagPose(boolean useRightCamera) {
        String llName = useRightCamera ? "limelight-right" : "limelight-left";

        if (LimelightHelpers.getTV(llName)){
            Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(llName);
            return Optional.of(targetPose);
        }

        return Optional.empty();
    }

    private boolean isRotated() {
        return alignmentHelpers.isRotated(closestReef, swerve.getPose());
    }

    @Override
    public void execute() {
        double vy = 0;
        double vx = 0;
        double rotation = alignmentHelpers.getRotation(closestReef, swerve.getPose());

        Optional<Pose3d> targetPose = getTagPose(true);
        if (targetPose.isPresent() && isRotated()) {
            Pose3d pose = targetPose.get();
            vy = tagPidController.calculate(pose.getX(), Constants.REEF_POLE_CENTER_OFFSET);
            SmartDashboard.putNumber("pac/TargetX", pose.getX());
            // vx = -tagPidController.calculate(pose.getZ(), 0.8);
        }
    
        SmartDashboard.putNumber("pac/velocityY", vy);

        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, rotation));
    }

    @Override
    public boolean isFinished() {
        if (!isRotated()) {
            return false;
        }

        Optional<Pose3d> targetPose = getTagPose(true);
        if (targetPose.isEmpty()) {
            return true;
        }
        Pose3d pose = targetPose.get();
        return Math.abs(pose.getX() - Constants.REEF_POLE_CENTER_OFFSET) < Constants.PRECISE_ALIGNMENT_POSITION_TOLERANCE;

    }
}
