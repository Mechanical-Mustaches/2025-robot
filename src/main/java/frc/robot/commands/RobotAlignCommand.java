package frc.robot.commands;

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


private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

    SwerveDriveSubsystem swerve;

    private PIDController pidController = new PIDController(0.015, 0.0005, 1);
    private PIDController wallPidController = new PIDController(0.005, 0, 2);
    private PIDController tagPidController = new PIDController(0.1, 0, 0);
    public RobotAlignCommand(SwerveDriveSubsystem swerve){
    this.swerve = swerve;
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
        if (Math.abs(distanceDifference) > 2 && distanceValidity){
            rotation = pidController.calculate(distanceDifference, 0);
        }
        if (Math.abs(310 - distanceToWall) > 10 && distanceValidity){
            vx = -wallPidController.calculate(distanceToWall,310);
        }
       
        
        //  double tagPosition = LimelightHelpers.getTX("limelight-right");
        //  double vy = 0;
        //  if (LimelightHelpers.getTV("limelight-right")){
        //      vy = tagPidController.calculate(tagPosition, -13);
        //  } else {

        //  }


   
          swerve.driveRobotRelative(new ChassisSpeeds(vx, 0, rotation));


        //  if(LimelightHelpers.getTV("limelight-right")){
        //      double xVelocity = pidController.calculate(LimelightHelpers.getTX("limelight-right"), 0.0);
        //      swerve.driveRobotRelative(new ChassisSpeeds(xVelocity * swerve.maximumSpeed, 0, 0));
        //  }else{
        //      swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        //  }
    }

   
    

 
}
