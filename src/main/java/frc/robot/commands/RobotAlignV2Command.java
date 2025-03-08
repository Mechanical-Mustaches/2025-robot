package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private PIDController pidController = new PIDController(0.015, 0.0005, 0);
    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(0.05, 0, 0);
    
    private DoubleSupplier horizontalInput;
    private final double rotationTolerance = 15;
    private final double distanceTolerance = 10;
    private final double tagTolerance = 0.6;
    private final double defaultVelocity;

    private final double wallDistanceSetpoint = 233;
    //previously 238
    private final double leftModeTagSetpoint = -17.1;
    private final double rightModeTagSetpoint = 18.1;

    public RobotAlignV2Command(SwerveDriveSubsystem swerve, DoubleSupplier horizontalInput, Mode mode){
        this.swerve = swerve;
        this.horizontalInput = horizontalInput;
        this.mode = mode;
        this.autoFinish = false;
        defaultVelocity = swerve.getMaximumChassisVelocity()/10;
    }

    public RobotAlignV2Command(SwerveDriveSubsystem swerve, Mode mode, boolean autoFinish){
        if (mode == Mode.manual){
            throw new IllegalArgumentException("must supply hotizontalInput in manual mode");
        }
        this.swerve = swerve;
        this.mode = mode;
        this.autoFinish = autoFinish;
        defaultVelocity = swerve.getMaximumChassisVelocity()/10;
    }   

    @Override
    public void initialize(){
        pidController.reset();
        wallPidController.reset();

    }
    private double getDistanceDifference(){
        return swerve.leftDistanceSensor.getRange() - swerve.rightDistanceSensor.getRange() - 15;
    }
    private double getDistanceToWall(){
        return (swerve.leftDistanceSensor.getRange() + swerve.rightDistanceSensor.getRange())/2;
    }
   
    @Override
    public void execute(){
        double distanceDifference = getDistanceDifference();
        // double rotation =
        double rotation = 0;
        double vx = 0;
        double distanceToWall = getDistanceToWall();
        boolean distanceValidity =
            swerve.leftDistanceSensor.getRange() > 130
            && swerve.rightDistanceSensor.getRange() > 130
            && swerve.leftDistanceSensor.isRangeValid()
            && swerve.rightDistanceSensor.isRangeValid();
        if (Math.abs(distanceDifference) > rotationTolerance && distanceValidity){
            rotation = pidController.calculate(distanceDifference, 0);
        }
        if (Math.abs(wallDistanceSetpoint - distanceToWall) > distanceTolerance && distanceValidity){
            vx = -wallPidController.calculate(distanceToWall, wallDistanceSetpoint);
            //previously 310
        }
       
         double vy = 0;
        if (mode == Mode.manual){
            vy = (horizontalInput.getAsDouble() * swerve.getMaximumChassisVelocity())/2;
        } else if(mode == Mode.left){
            if (LimelightHelpers.getTV("limelight-right")){
                double tagPosition = LimelightHelpers.getTX("limelight-right");
                if (tagPosition > 0){
                    vy = defaultVelocity;
                }
                else if (Math.abs(tagPosition - leftModeTagSetpoint) > tagTolerance){
                    vy = tagPidController.calculate(tagPosition, leftModeTagSetpoint);
                }
            } else{
                vy = defaultVelocity;
            }
        } else if(mode == Mode.right){
            if (LimelightHelpers.getTV("limelight-left")){
                double tagPosition = LimelightHelpers.getTX("limelight-left");
                if (tagPosition < 0){
                    vy = -defaultVelocity;
                }
                else if (Math.abs(tagPosition - rightModeTagSetpoint) > tagTolerance){
                    vy = tagPidController.calculate(tagPosition, rightModeTagSetpoint);
                }
            } else{
                vy = -defaultVelocity;
            }
        }

   
          swerve.driveRobotRelative(new ChassisSpeeds(0, 0, rotation));


    }

   @Override
   public boolean isFinished(){
       if (!autoFinish){
        return false;
       }
       boolean isRotated = (Math.abs(getDistanceDifference()) < rotationTolerance);
       boolean isVerticallyAligned = Math.abs(wallDistanceSetpoint - getDistanceToWall()) < distanceTolerance;
       boolean isHorizontallyAlignedLeft = this.mode == Mode.left && 
       LimelightHelpers.getTV("limelight-right") &&
       Math.abs(LimelightHelpers.getTX("limelight-right") - leftModeTagSetpoint) < tagTolerance;
       
       boolean isHorizontallyAlignedRight = this.mode == Mode.right && 
       LimelightHelpers.getTV("limelight-left") &&
       Math.abs(LimelightHelpers.getTX("limelight-left") - rightModeTagSetpoint) < tagTolerance;

       return isRotated && isVerticallyAligned && (isHorizontallyAlignedLeft || isHorizontallyAlignedRight);

   }
    

 
}
