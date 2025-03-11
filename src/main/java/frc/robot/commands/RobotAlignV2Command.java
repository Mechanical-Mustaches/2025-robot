package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignV2Command extends Command{

    public enum Mode{
        left,
        right,
        manual
    }
    private Mode mode;

    SwerveDriveSubsystem swerve;
    private boolean autoFinish;
    

    private PIDController pidRotation = new PIDController(0.1, 0.005, 0);
    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(5, 0.01, 0);
    
    private static final double reefPoleCenterOffset = 0.1558544;

    private DoubleSupplier horizontalInput;
    private final double rotationTolerance = 15;
    private final double distanceTolerance = 10;
    private final double tagTolerance = 0.6;
    private final double defaultVelocity;

    private final double wallDistanceSetpoint = 400;

    private final double leftModeTagSetpoint = -17.1;
    private final double rightModeTagSetpoint = 18.1;

    public RobotAlignV2Command(SwerveDriveSubsystem swerve){
        this.swerve = swerve;
      //  this.mode = mode;
        this.autoFinish = false;
        defaultVelocity = swerve.getMaximumChassisVelocity()/10;
    }

    @Override
    public void initialize(){
        pidRotation.reset();
        wallPidController.reset();
        tagPidController.reset();

    }

    private double getDistanceToWall(){
        return Math.min(swerve.leftDistanceSensor.getRange(), swerve.rightDistanceSensor.getRange());
    }
   
    @Override
    public void execute(){
        double rotation;
        double vy = 0;
        double vx= 0;
        SwerveDriveSubsystem.ReefPosition closestReef = swerve.getClosestReefPosition();
        double desiredPositiveAngle = closestReef.rotation().getDegrees();
        double desiredNegativeAngle = closestReef.rotation().getDegrees() - 360;
        double currentAngle = swerve.getPose().getRotation().getDegrees();
        if (Math.abs(desiredPositiveAngle - currentAngle) > Math.abs(desiredNegativeAngle - currentAngle)){
            rotation = pidRotation.calculate(currentAngle, desiredNegativeAngle);
        } else {
            rotation = pidRotation.calculate(currentAngle, desiredPositiveAngle);
        }
        
        if (LimelightHelpers.getTV("limelight-right")){
            Pose3d leftTargetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-right");
            vy = tagPidController.calculate(leftTargetPose.getX(), reefPoleCenterOffset);
            SmartDashboard.putNumber("rav/TargetX", leftTargetPose.getX());
        }

        double distanceToWall = getDistanceToWall();
        boolean distanceValidity =
            swerve.leftDistanceSensor.getRange() > 130
            && swerve.rightDistanceSensor.getRange() > 130
            && swerve.leftDistanceSensor.isRangeValid()
            && swerve.rightDistanceSensor.isRangeValid();
         if (Math.abs(wallDistanceSetpoint - distanceToWall) > distanceTolerance && distanceValidity){
                vx = -wallPidController.calculate(distanceToWall, wallDistanceSetpoint);
         }

        
        SmartDashboard.putNumber("rav/currentRotation", currentAngle);
        SmartDashboard.putNumber("rav/desiredNegativeRotation",  desiredNegativeAngle);
        SmartDashboard.putNumber("rav/desiredPositiveRotation",  desiredPositiveAngle);
        SmartDashboard.putNumber("rav/velocityY", vy);
        swerve.driveRobotRelative(new ChassisSpeeds(0, vy, rotation));
    }

//    @Override
//    public boolean isFinished(){
//        if (!autoFinish){
//         return false;
//        }
//        boolean isRotated = (Math.abs(getDistanceDifference()) < rotationTolerance);
//        boolean isVerticallyAligned = Math.abs(wallDistanceSetpoint - getDistanceToWall()) < distanceTolerance;
//        boolean isHorizontallyAlignedLeft = this.mode == Mode.left && 
//        LimelightHelpers.getTV("limelight-right") &&
//        Math.abs(LimelightHelpers.getTX("limelight-right") - leftModeTagSetpoint) < tagTolerance;
       
//        boolean isHorizontallyAlignedRight = this.mode == Mode.right && 
//        LimelightHelpers.getTV("limelight-left") &&
//        Math.abs(LimelightHelpers.getTX("limelight-left") - rightModeTagSetpoint) < tagTolerance;

//        return isRotated && isVerticallyAligned && (isHorizontallyAlignedLeft || isHorizontallyAlignedRight);

//    }
    

 
}
