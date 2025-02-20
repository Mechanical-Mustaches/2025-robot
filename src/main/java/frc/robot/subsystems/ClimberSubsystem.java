package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    private SparkMax climber = new SparkMax(19, MotorType.kBrushless);
  
    public ClimberSubsystem(){
        SparkMaxConfig config1 = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
            .pid(0.1,0.00001,0.0001)
            .maxOutput(.5)
            .minOutput(-.5);

        config1
            .smartCurrentLimit(50)
            .apply(pidConfig)
            .inverted(true)
            .idleMode(IdleMode.kBrake);

        climber.configure(config1, null, null);
    }

    public double getEncoderValue(){
        return climber.getEncoder().getPosition();
    }

    public void climber1(){
        climber.getClosedLoopController().setReference(8, ControlType.kPosition);
        //previously 780
    }

    public void climber2(){
        climber.getClosedLoopController().setReference(2500, ControlType.kPosition);  
    }
    public void climber0(){
        climber.getClosedLoopController().setReference(0, ControlType.kPosition);
    }
     
     public void dumbClimb(){
         climber.set(-0.2);
     }

    public void resetEncoder(){
        climber.getEncoder().setPosition(0);
    }

    public void climberStop(){
        climber.set(0);
    }
    public boolean isDone(){
        if(climber.getEncoder().getPosition() > 11.9){
            return true;
        } else {
            return false;
        }
    }
}

    
