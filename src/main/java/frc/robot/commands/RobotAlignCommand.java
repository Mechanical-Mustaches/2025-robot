package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotAlignCommand extends Command{

    SwerveDriveSubsystem swerve;
    private final PIDController pidController = new PIDController(0.01, 0, 1);

    public RobotAlignCommand(SwerveDriveSubsystem swerve){
    this.swerve = swerve;
    }

    @Override
    public void execute(){
        double distanceDifference = swerve.leftDistanceSensor.getRange() - swerve.rightDistanceSensor.getRange() - 20;
        double rotation = pidController.calculate(distanceDifference, 0);
        swerve.driveRobotRelative(new ChassisSpeeds(0, 0, rotation));
        // if(LimelightHelpers.getTV("limelight-right")){
        //     double xVelocity = pidController.calculate(LimelightHelpers.getTX("limelight-right"), 0.0);
        //     swerve.driveRobotRelative(new ChassisSpeeds(xVelocity * swerve.maximumSpeed, 0, 0));
        // }else{
        //     swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        // }
    }

   
    

 
}
