package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase {
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(22, MotorType.kBrushless);
    private boolean intakingAlgae = false;

    public AlgaeHandlerSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        intakeConfig
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kBrake);

        pivotConfig
        .smartCurrentLimit(10)
        .idleMode(IdleMode.kBrake);

        pivotConfig
                .smartCurrentLimit(10)
                .idleMode(IdleMode.kBrake);

    }

    public double getEncoderValue() {
        return pivot.getEncoder().getPosition();
    }

    public void pivot() {
        if (isAlgaeDetected() > 5) {
            pivot.getClosedLoopController().setReference(0, ControlType.kPosition);
        }
    }

    public void stopPivot() {
        pivot.set(0);
    }


    public void pivotUp(){
        if(getEncoderValue()>=0.4){
           pivot.set(-0.05);
           } else{
               stopPivot();
           }
    }

    public void pivotDown(){
        if(getEncoderValue()<=0.05){
        pivot.set(0.05);
        } else{

            stopPivot();
        }

    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(0);
    }

    public void intake() {
        intakingAlgae = true;
        if (intakeActivator.getOutputCurrent() < 5) {
            intakeActivator.set(1);
        } else {
            intakeActivator.set(0.2);
        }
    }

    public void stopIntake() {
        intakeActivator.set(0);
        intakingAlgae = false;
    }

    public double isAlgaeDetected() {
        if (intakeActivator.getOutputCurrent() > 15) {
            return intakeActivator.getOutputCurrent() - 6;
        } else {
            return 0;
        }
    }

    public boolean isIntakingAlgae() {
        return intakingAlgae;
    }





    public void dumbPivot(double speed){
        if(speed>0){
            if(getEncoderValue() >= 0.4){
                stopPivot();
            } else {
                pivot.set(speed);
            }

        } else{
            if(getEncoderValue() <= 0.05){
                stopPivot();
            } else {
                pivot.set(speed);
            }
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algaeDetectionValue", isAlgaeDetected());
        SmartDashboard.putNumber("pivotEncoderValue", pivot.getEncoder().getPosition());
        SmartDashboard.putBoolean("intakingAlgae", intakingAlgae);

    }

}
