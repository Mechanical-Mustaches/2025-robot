package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase{
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);

    //update motors

    public AlgaeHandlerSubsystem(){
        SparkMaxConfig intakeConfig = new SparkMaxConfig();


       
        intakeConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);


    }

    public void intake(){
        intakeActivator.set(-1);
    }
   /*  public void reverseIntake(){
        intakeActivator.set(1);
    }
        */
    public void stopIntake(){
        intakeActivator.set(0);
    }
        
   
   

}
