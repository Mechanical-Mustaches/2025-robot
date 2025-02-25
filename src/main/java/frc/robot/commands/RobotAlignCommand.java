package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignCommand extends Command{

    public enum Mode{
        left,
        right,
        manual
    }
    private Mode mode;

    SwerveDriveSubsystem swerve;

    private PIDController pidController = new PIDController(0.015, 0.0005, 0);
    private PIDController wallPidController = new PIDController(0.005, 0, 0);
    private PIDController tagPidController = new PIDController(0.05, 0, 0);
    
    private DoubleSupplier horizontalInput;

    public RobotAlignCommand(SwerveDriveSubsystem swerve, DoubleSupplier horizontalInput, Mode mode){
        this.swerve = swerve;
        this.horizontalInput = horizontalInput;
        this.mode = mode;
    }

    public RobotAlignCommand(SwerveDriveSubsystem swerve, Mode mode){
        if (mode == Mode.manual){
            throw new IllegalArgumentException("must supply hotizontalInput in manual mode");
        }
        this.swerve = swerve;
        this.mode = mode;
    }   

    @Override
    public void initialize(){
        pidController.reset();
        wallPidController.reset();

    }

   
    @Override
    public void execute(){
        double distanceDifference = swerve.leftDistanceSensor.getRange() - swerve.rightDistanceSensor.getRange() - 20;
        // double rotation =
        double rotation = 0;
        double vx = 0;
        double distanceToWall = (swerve.leftDistanceSensor.getRange() + swerve.rightDistanceSensor.getRange())/2;
        boolean distanceValidity = swerve.leftDistanceSensor.getRange() > 0 && swerve.rightDistanceSensor.getRange() > 0;
        if (Math.abs(distanceDifference) > 15 && distanceValidity){
            rotation = pidController.calculate(distanceDifference, 0);
        }
        if (Math.abs(250 - distanceToWall) > 20 && distanceValidity){
            vx = -wallPidController.calculate(distanceToWall,250);
            //previously 310
        }
       
         double vy = 0;
        if (mode == Mode.manual){
            vy = (horizontalInput.getAsDouble() * swerve.getMaximumChassisVelocity())/2;
        } else if(mode == Mode.left){
            if (LimelightHelpers.getTV("limelight-right")){
                double tagPosition = LimelightHelpers.getTX("limelight-right");
                if (tagPosition > 0){
                    vy = swerve.getMaximumChassisVelocity()/10;
                }
                else if (Math.abs(tagPosition + 17.1) > 0.6){
                    vy = tagPidController.calculate(tagPosition, -17.1);
                }
            } else{
                vy = swerve.getMaximumChassisVelocity()/10;
            }
        } else if(mode == Mode.right){
            if (LimelightHelpers.getTV("limelight-left")){
                double tagPosition = LimelightHelpers.getTX("limelight-left");
                if (tagPosition < 0){
                    vy = -swerve.getMaximumChassisVelocity()/10;
                }
                else if (Math.abs(tagPosition - 10.7) > 0.6){
                    vy = tagPidController.calculate(tagPosition, 10.7);
                }
            } else{
                vy = -swerve.getMaximumChassisVelocity()/10;
            }
        }
       


   
          swerve.driveRobotRelative(new ChassisSpeeds(vx, vy, rotation));


        //  if(LimelightHelpers.getTV("limelight-right")){
        //      double xVelocity = pidController.calculate(LimelightHelpers.getTX("limelight-right"), 0.0);
        //      swerve.driveRobotRelative(new ChassisSpeeds(xVelocity * swerve.maximumSpeed, 0, 0));
        //  }else{
        //      swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        //  }
    }

   
    

 
}
