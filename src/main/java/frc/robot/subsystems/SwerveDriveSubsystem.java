package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {
    SwerveDrive swerveDrive;
    TimeOfFlight leftDistanceSensor = new TimeOfFlight(31);
    TimeOfFlight rightDistanceSensor = new TimeOfFlight(30);
    // maximumSpeed in meters per second.
    public double maximumSpeed = 5.3;

    public SwerveDriveSubsystem() {

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

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                    angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }
    @Override
    public void periodic(){
        LimelightHelpers.PoseEstimate limelightPoseRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
        LimelightHelpers.PoseEstimate limelightPoseLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

        SmartDashboard.putNumber("# of tags right", limelightPoseRight.tagCount);
        SmartDashboard.putNumber("# of tags left", limelightPoseLeft.tagCount);
        SmartDashboard.putNumber("tag span right", limelightPoseRight.tagSpan);
        SmartDashboard.putNumber("tag span left", limelightPoseLeft.tagSpan);
        SmartDashboard.putNumber("tag distance right", limelightPoseRight.avgTagDist);
        SmartDashboard.putNumber("tag distance left", limelightPoseLeft.avgTagDist);
        SmartDashboard.putNumber("tag area right", limelightPoseRight.avgTagArea);
        SmartDashboard.putNumber("tag area left", limelightPoseLeft.avgTagArea);


        if(limelightPoseLeft.tagCount >= 2 && limelightPoseLeft.avgTagDist <= 5){
            swerveDrive.addVisionMeasurement(limelightPoseLeft.pose, limelightPoseLeft.timestampSeconds);
        }
        if(limelightPoseRight.tagCount >= 2 && limelightPoseRight.avgTagDist <= 5){
            swerveDrive.addVisionMeasurement(limelightPoseRight.pose, limelightPoseRight.timestampSeconds);
        }

        SmartDashboard.putNumber("leftDistanceFromReef", leftDistanceSensor.getRange());
        SmartDashboard.putNumber("rightDistanceFromReef", rightDistanceSensor.getRange());
    }
}
