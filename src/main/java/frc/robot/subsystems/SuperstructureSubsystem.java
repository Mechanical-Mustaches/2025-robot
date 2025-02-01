package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase{
    private SparkMax leftPivot = new SparkMax(20,MotorType.kBrushless);
    private SparkMax rightPivot = new SparkMax(21,MotorType.kBrushless);
    

    public SuperstructureSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

            

        leftConfig
            .smartCurrentLimit(10)
            .idleMode(IdleMode.kBrake);
        
        rightConfig
            .apply(leftConfig)
            .follow(leftPivot, true);
            


            leftPivot.configure(leftConfig,null,null);
            rightPivot.configure(leftConfig,null,null);
    }

    public void pivotOut(){
        leftPivot.set(.2);
    }

    public void pivotStop(){
        leftPivot.set(0);
    }
}
