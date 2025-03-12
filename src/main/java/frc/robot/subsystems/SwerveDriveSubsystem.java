package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    Pose2d pose2d;
    Rotation2d rotation2d;
    public TimeOfFlight leftDistanceSensor = new TimeOfFlight(31);
    public TimeOfFlight rightDistanceSensor = new TimeOfFlight(30);
    // maximumSpeed in meters per second.
    public double maximumSpeed = 5.3;
    private final Field2d m_field = new Field2d();

    private double xPose;
    private double yPose;
    public double closestReef;

    /**
     * Represents the desired position and rotation of the robot for a given
     * reef position.
     */
    public record ReefPosition(Translation2d translation, Rotation2d rotation, String label) {
        static final ReefPosition[] positions = {
                new ReefPosition(new Translation2d(14.373249, 4.025900), Rotation2d.fromDegrees(180), "Red 1"),
                new ReefPosition(new Translation2d(13.716061, 2.887662), Rotation2d.fromDegrees(240), "Red 2"),
                new ReefPosition(new Translation2d(12.401724, 2.887685), Rotation2d.fromDegrees(300), "Red 3"),
                new ReefPosition(new Translation2d(11.744576, 4.025946), Rotation2d.fromDegrees(0), "Red 4"),
                new ReefPosition(new Translation2d(12.401764, 5.164184), Rotation2d.fromDegrees(60), "Red 5"),
                new ReefPosition(new Translation2d(13.716101, 5.164161), Rotation2d.fromDegrees(120), "Red 6"),
                new ReefPosition(new Translation2d(3.175000, 4.025900), Rotation2d.fromDegrees(0), "Blue 1"),
                new ReefPosition(new Translation2d(3.832148, 5.164161), Rotation2d.fromDegrees(0), "Blue 2"),
                new ReefPosition(new Translation2d(5.146485, 5.164184), Rotation2d.fromDegrees(0), "Blue 3"),
                new ReefPosition(new Translation2d(5.803674, 4.025946), Rotation2d.fromDegrees(0), "Blue 4"),
                new ReefPosition(new Translation2d(5.146525, 2.887685), Rotation2d.fromDegrees(0), "Blue 5"),
                new ReefPosition(new Translation2d(3.832188, 2.887662), Rotation2d.fromDegrees(0), "Blue 6"),
        };

        @Override
        public final String toString() {
            return this.label;
        }
    }

    public SwerveDriveSubsystem() {
        SmartDashboard.putData("Field", m_field);
        leftDistanceSensor.setRangeOfInterest(0, 6, 15, 10);
        rightDistanceSensor.setRangeOfInterest(0, 6, 15, 10);
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        } catch (IOException e) {

            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given
                                                                          // ROBOT RELATIVE ChassisSpeeds. Also
                                                                          // optionally outputs individual module
                                                                          // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d initialPose) {
        swerveDrive.resetOdometry(initialPose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public void resetGyro() {
        swerveDrive.zeroGyro();
    }

    public double getMaximumChassisVelocity() {
        return this.swerveDrive.getMaximumChassisVelocity();
    }

    /**
     * Returns the inversion multiplier for the robot controls based on the
     * current alliance.
     * 
     * @return 1 for the Blue alliance, or -1 for the Red alliance
     */
    private int getInversion() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return -1;
        }

        return 1;
    }

    public Command driveCommand(
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            var maximumVelocity = swerveDrive.getMaximumChassisVelocity() * getInversion();
            var x = translationX.getAsDouble() * maximumVelocity;
            var y = translationY.getAsDouble() * maximumVelocity;
            var translation = new Translation2d(x, y);
            var rotation = angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity();

            swerveDrive.drive(translation, rotation, true, false);
        });
    }

    /**
     * Calculates the closest ReefPosition to the current pose.
     * 
     * @return The closest ReefPosition
     */
    public ReefPosition getClosestReefPosition() {
        ReefPosition closestPosition = null;

        for (ReefPosition position : ReefPosition.positions) {
            if (closestPosition == null) {
                closestPosition = position;
            } else {
                double closestDistance = getPose().getTranslation().getDistance(closestPosition.translation);
                double measuredDistance = getPose().getTranslation().getDistance(position.translation);

                if (closestDistance > measuredDistance) {
                    closestPosition = position;
                }
            }
        }

        return closestPosition;
    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation("limelight-right", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-left", getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        xPose = swerveDrive.getPose().getX();
        yPose = swerveDrive.getPose().getY();

        LimelightHelpers.PoseEstimate limelightPoseRight = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
        LimelightHelpers.PoseEstimate limelightPoseLeft = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

        if (limelightPoseRight != null) {
            Pose2d pose = limelightPoseRight.pose;
            if (limelightPoseRight.avgTagDist <= 5 && pose.getX() > 0 && pose.getY() > 0) {
                swerveDrive.addVisionMeasurement(pose, limelightPoseRight.timestampSeconds);
            }
        }
        if (limelightPoseLeft != null) {
            Pose2d pose = limelightPoseLeft.pose;
            if (limelightPoseLeft.avgTagDist <= 5 && pose.getX() > 0 && pose.getY() > 0) {
                swerveDrive.addVisionMeasurement(pose, limelightPoseLeft.timestampSeconds);
            }
        }
        m_field.setRobotPose(getPose());

        Pose3d leftTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-left");

        SmartDashboard.putNumber("ll-left/x", leftTargetPose.getX());
        SmartDashboard.putNumber("ll-left/y", leftTargetPose.getY());
        SmartDashboard.putNumber("ll-left/z", leftTargetPose.getZ());

        SmartDashboard.putNumber("XPose", xPose);
        SmartDashboard.putNumber("YPose", yPose);

        SmartDashboard.putString("ClosestReef", getClosestReefPosition().toString());

        SmartDashboard.putNumber("leftDistanceFromReef", leftDistanceSensor.getRange());
        SmartDashboard.putNumber("rightDistanceFromReef", rightDistanceSensor.getRange());
        SmartDashboard.putNumber("distanceSensorSampleRate", leftDistanceSensor.getSampleTime());
        SmartDashboard.putBoolean("leftDistanceValid", leftDistanceSensor.isRangeValid());
        SmartDashboard.putBoolean("rightDistanceValid", rightDistanceSensor.isRangeValid());
    }
}
