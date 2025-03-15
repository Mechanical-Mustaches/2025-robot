package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RoughAlignCommand extends Command {
    // Distance from center point of reef to end command and switch to precise
    // alignment
    public static final double FINISH_DISTANCE = 0.5;

    private AlignmentHelpers alignmentHelpers = new AlignmentHelpers();
    private PIDController roughPidController = new PIDController(1, 0, 0);

    private SwerveDriveSubsystem swerve;
    private SwerveDriveSubsystem.ReefPosition closestReef;

    private final double roughAlignOffset = 0.4;

    public RoughAlignCommand(SwerveDriveSubsystem swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        alignmentHelpers.initialize();
        closestReef = swerve.getClosestReefPosition();
    }

    @Override
    public void execute() {
        double vy = 0;
        double vx = 0;
        double reefAngle = closestReef.rotation().getRadians() - Math.PI;
        double currentPositionX = swerve.getPose().getX();
        double currentPositionY = swerve.getPose().getY();
        double desiredPositionX = closestReef.translation().getX() + roughAlignOffset * Math.cos(reefAngle);
        double desiredPositionY = closestReef.translation().getY() + roughAlignOffset * Math.sin(reefAngle);
        vx = roughPidController.calculate(currentPositionX, desiredPositionX);
        vy = roughPidController.calculate(currentPositionY, desiredPositionY);
        double rotation = alignmentHelpers.getRotation(closestReef, swerve.getPose());

        SmartDashboard.putNumber("align/rough/currentPositionX", currentPositionX);
        SmartDashboard.putNumber("align/rough/currentPositionY", currentPositionY);
        SmartDashboard.putNumber("align/rough/desiredPositionX", desiredPositionX);
        SmartDashboard.putNumber("align/rough/desiredPositionY", desiredPositionY);

        swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, rotation));
    }

    @Override
    public boolean isFinished() {
        Translation2d robotTranslation = swerve.getPose().getTranslation();
        Translation2d reefTranslation = closestReef.translation();
        return robotTranslation.getDistance(reefTranslation) < FINISH_DISTANCE;
    }
}
