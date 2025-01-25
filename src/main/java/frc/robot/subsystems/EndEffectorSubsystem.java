package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
    private SparkMax effectorLeft = new SparkMax(13, MotorType.kBrushless);
    private SparkMax effectorRight = new SparkMax(14, MotorType.kBrushless);
    

    public EndEffectorSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        rightConfig
            .apply(leftConfig)
            .follow(16);
        
    }
    public void effectorSpin1(){
        effectorLeft.set(1);
    }
    public void effectorSpin2(){
        effectorLeft.set(0.5);
    }
    public void effectorSpin3(){
        effectorLeft.set(0.25);
    }
    public void effectorStop(){
        effectorLeft.set(0);
    }
}
