package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase{
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax intakePivoter = new SparkMax(22, MotorType.kBrushless);

    public AlgaeHandlerSubsystem(){
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();
        SparkMaxConfig intakeConfig = new SparkMaxConfig();


        pidConfig
            .pid(0.01, 0, 0);

        intakeConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);


        pivotConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig);
            
        intakePivoter.configure(pivotConfig, null, null);
    }

    public void intake(){
        intakeActivator.set(-1);
    }
    public void reverseIntake(){
        intakeActivator.set(1);
    }
    public void stopIntake(){
        intakeActivator.set(0);
    }
    public void pivotOut(){
        intakePivoter.set(0.2);
    }
    public void pivotIn(){
        intakePivoter.set(-0.2);
    }
    public double getEncoderValue(){
        return intakePivoter.getEncoder().getPosition();
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("AlgaePivotEncoderValue", getEncoderValue());
    }

}
