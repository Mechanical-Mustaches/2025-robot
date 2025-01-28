package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    private SparkMax climber1 = new SparkMax(18 , MotorType.kBrushless);
    private SparkMax climber2 = new SparkMax(19, MotorType.kBrushless);
  
    public ClimberSubsystem(){
        SparkMaxConfig config1 = new SparkMaxConfig();
        SparkMaxConfig config2 = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
            .pid(0.01,0,0)
            .maxOutput(.5)
            .minOutput(.2);

        config1
            .smartCurrentLimit(50)
            .apply(pidConfig)
            .idleMode(IdleMode.kBrake);
        config2
            .apply(config1)
            .follow(13, true);

        climber1.configure(config1, null, null);
        climber2.configure(config2, null, null);
    }

    public double getEncoderValue(){
        return climber1.getEncoder().getPosition();
    }

    public void verticleClimber(){
        climber1.set(0.2);
    }

    public void angledClimber(){
        climber1.set(-0.2);
    }

    public void climberStop(){
        climber1.set(0);
    }
}

    
