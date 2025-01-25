package frc.robot.subsystems;

import javax.sound.sampled.SourceDataLine;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ClimberSubsystem extends SubsystemBase {
    private RelativeEncoder encoderValue; 
    
    private SparkMax climber1 = new SparkMax(18 , MotorType.kBrushless);
    private SparkMax climber2 = new SparkMax(19, MotorType.kBrushless);
  
    public ClimberSubsystem(){
        
    }

    private double distance = encoderValue.getPosition();
// need to use shuffleboard? to see how encoder position works

    public void verticleClimber(){
        climber1.set(distance * 0.5);
        climber2.set(distance * -0.5);
    }

    public void angledClimber(){
        climber1.set(distance * -0.5);
        climber2.set(distance * 0.5);
    }

    
}
