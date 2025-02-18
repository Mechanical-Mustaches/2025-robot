package frc.robot.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    UsbCamera climberCamera = new UsbCamera("climberCamera", 0);
    

    public CameraSubsystem(){
        climberCamera.setVideoMode(PixelFormat.kBGR, 25, 25, 10);

    }
}

// @Override
// public void periodic(){
//    CameraServer.putVideo("climberCamera", 25, 25);
// }
