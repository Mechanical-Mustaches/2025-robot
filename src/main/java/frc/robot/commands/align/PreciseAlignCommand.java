package frc.robot.commands.align;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.commands.align.Constants.Mode;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PreciseAlignCommand extends Command {
    private SwerveDriveSubsystem swerve;
    private AlignmentHelpers alignmentHelpers = new AlignmentHelpers();

    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(3, 0.03, 0);

    SwerveDriveSubsystem.ReefPosition closestReef;
    Constants.Mode mode;

    public PreciseAlignCommand(SwerveDriveSubsystem swerve, Constants.Mode mode) {
        this.swerve = swerve;
        this.mode = mode;
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

    private Optional<Pose3d> getTagPose() {
        String llName = this.mode == Constants.Mode.LEFT ? "limelight-right" : "limelight-left";

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

        Optional<Pose3d> targetPose = getTagPose();
        if (targetPose.isPresent() && isRotated()) {
            Pose3d pose = targetPose.get();
            if (mode == Mode.LEFT){
                vy = tagPidController.calculate(pose.getX(), Constants.REEF_POLE_CENTER_OFFSET);
            } else if (mode == Mode.RIGHT){
                vy = tagPidController.calculate(pose.getX(), -Constants.REEF_POLE_CENTER_OFFSET);
            }
            
            SmartDashboard.putNumber("pac/TargetX", pose.getX());
            vx = -tagPidController.calculate(pose.getZ(), 0.45)/2;
        }
    
        SmartDashboard.putNumber("pac/velocityY", vy);

        swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, rotation));
    }

    @Override
    public boolean isFinished() {
        if (!isRotated()) {
            return false;
        }

        Optional<Pose3d> targetPose = getTagPose();
        if (targetPose.isEmpty()) {
            return true;
        }
        // Pose3d pose = targetPose.get();
        // if (mode == Mode.LEFT){
        //     return Math.abs(pose.getX() - Constants.REEF_POLE_CENTER_OFFSET) < Constants.PRECISE_ALIGNMENT_POSITION_TOLERANCE;
        // } else if (mode == Mode.RIGHT){
        //     return Math.abs(pose.getX() + Constants.REEF_POLE_CENTER_OFFSET) < Constants.PRECISE_ALIGNMENT_POSITION_TOLERANCE;
        // }
        return false;

    }
}
