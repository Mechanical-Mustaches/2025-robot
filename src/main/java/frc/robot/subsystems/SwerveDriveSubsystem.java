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
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
    private double rotation;
    public double closestReef;

    public enum ReefPosition {
        Red1(14.326,4.6,180,"Red 1"),
        Red2(13.73,5.55,240,"Red 2"),
        Red3(12.55,5.37,300,"Red 3"),
        Red4(11.837,3.98,0,"Red 4"),
        Red5(12.5,3.33,60,"Red 5"),
        Red6(13.7,3.35,120,"Red 6"),
        Blue1(0,0,0,"Blue 1"),
        Blue2(0,0,0,"Blue 2"),
        Blue3(0,0,0,"Blue 3"),
        Blue4(0,0,0,"Blue 4"),
        Blue5(0,0,0,"Blue 5"),
        Blue6(0,0,0,"Blue 6");

        public final Translation2d translation;
        public final Rotation2d rotation;
        private final String label;

        private ReefPosition(double x, double y, double angle, String label) {
            this.translation = new Translation2d(x,y);
            this.rotation = Rotation2d.fromDegrees(angle);
            this.label = label;
        }

        @Override
        public String toString() {
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
 
    public void resetGyro(){
        swerveDrive.zeroGyro();
    }


    public double getMaximumChassisVelocity() { return this.swerveDrive.getMaximumChassisVelocity(); }

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

    public ReefPosition getClosestReefPosition(){
        ReefPosition closestPosition = null;

        for (ReefPosition position:ReefPosition.values()) {
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
    public void periodic(){
        LimelightHelpers.PoseEstimate limelightPoseRight = null;
        // LimelightHelpers.PoseEstimate limelightPoseLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

         xPose = swerveDrive.getPose().getX();
         yPose = swerveDrive.getPose().getY();

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()){
            if (ally.get() == Alliance.Red && !DriverStation.isAutonomous()){
                limelightPoseRight = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-right");
            }
            if (ally.get() == Alliance.Blue || DriverStation.isAutonomous()){
                limelightPoseRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
            }
        }
        
        if (limelightPoseRight != null){
            if(limelightPoseRight.tagCount >= 2 && limelightPoseRight.avgTagDist <= 5){
                swerveDrive.addVisionMeasurement(limelightPoseRight.pose, limelightPoseRight.timestampSeconds);
            }
        }

        

        SmartDashboard.putNumber("XPose", xPose);
        SmartDashboard.putNumber("YPose", yPose);

       SmartDashboard.putString("ClosestReef", getClosestReefPosition().toString());


        SmartDashboard.putNumber("right april tag position", LimelightHelpers.getTX("limelight-right"));
        SmartDashboard.putNumber("left april tag position", LimelightHelpers.getTX("limelight-left"));
        SmartDashboard.putBoolean("april tag TV", LimelightHelpers.getTV("limelight-right"));

        // if(limelightPoseLeft.tagCount >= 2 && limelightPoseLeft.avgTagDist <= 5){
        //     swerveDrive.addVisionMeasurement(limelightPoseLeft.pose, limelightPoseLeft.timestampSeconds);
        // }
     

        SmartDashboard.putNumber("leftDistanceFromReef", leftDistanceSensor.getRange());
        SmartDashboard.putNumber("rightDistanceFromReef", rightDistanceSensor.getRange());
        SmartDashboard.putNumber("distanceSensorSampleRate", leftDistanceSensor.getSampleTime());
        SmartDashboard.putBoolean("leftDistanceValid", leftDistanceSensor.isRangeValid());
        SmartDashboard.putBoolean("rightDistanceValid", rightDistanceSensor.isRangeValid());

        m_field.setRobotPose(getPose());

    }
}
