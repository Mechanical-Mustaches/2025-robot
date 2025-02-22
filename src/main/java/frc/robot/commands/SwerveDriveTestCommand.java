package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveTestCommand extends Command {
    SwerveDriveSubsystem swerveDrive;
    double rotation;
    double vx;
    double vy;
    
    public SwerveDriveTestCommand(SwerveDriveSubsystem swerve, double rotation, double vx, double vy){
        this.swerveDrive = swerve;
        this.rotation = rotation;
        this.vx = vx;
        this.vy = vy;
    }  

    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
       swerveDrive.driveRobotRelative(new ChassisSpeeds(vx,vy,rotation));
    }

  
}
