package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase{
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(22, MotorType.kBrushless);

    

    public AlgaeHandlerSubsystem(){
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
        .pid(0.5, 0.00001, 0)
        .maxOutput(.2)
        .minOutput(-.2);

        intakeConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);

        pivotConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);


    }

    public double getEncoderValue(){
        return pivot.getEncoder().getPosition();
    }

    public void horizontalPivot(){
        pivot.getClosedLoopController().setReference(0, ControlType.kPosition);
    }

    public void verticalPivot(){
        pivot.getClosedLoopController().setReference(7, ControlType.kPosition);  
    }

    public void stopPivot(){
        pivot.set(0);
    }

    public void pivotUp(){
        pivot.set(.2);
    }

    public void pivotDown(){
        pivot.set(-.2);
    }

    public void intake(){
        intakeActivator.set(1);
    }

   /*  public void reverseIntake(){
        intakeActivator.set(1);
    }
        */
    public void stopIntake(){
        intakeActivator.set(0);
    }
    public double isAlgaeDetected(){
        if (intakeActivator.getOutputCurrent() > 5){
            return intakeActivator.getOutputCurrent() -6;
        }else{
            return 0;
        }
    }
   
    

}
