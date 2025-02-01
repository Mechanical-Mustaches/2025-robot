package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase{
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax intakePivoter = new SparkMax(22, MotorType.kBrushless);

    public AlgaeHandlerSubsystem(){
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();


        pidConfig
            .pid(0.01, 0, 0);

        pivotConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig);
            
        intakeActivator.configure(pivotConfig, null, null);
    }

    public void intake(){
        intakeActivator.set(0.05);
    }
    public void reverseIntake(){
        intakeActivator.set(-0.05);
    }
    public void stopIntake(){
        intakeActivator.set(0);
    }
    public void pivotOut(){
        intakePivoter.getClosedLoopController().setReference(0.2, ControlType.kPosition);
    }
    public void pivotIn(){
        intakePivoter.getClosedLoopController().setReference(0, ControlType.kPosition);
    }

}
