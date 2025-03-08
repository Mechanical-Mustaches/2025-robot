package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    
    private SparkMax climber = new SparkMax(19, MotorType.kBrushless);
    
    public enum Stage{
        S1(1650),
        S2(3950 );
        //previously 4000
        public final double encoderValue;

        private Stage(double stage){
            this.encoderValue = stage;
        }

    }
    
    
    public ClimberSubsystem(){
        SparkMaxConfig config1 = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
            .pid(0.2,0.0000,0.000)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        config1
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig);

        climber.configure(config1, null, null);
    }

    public double getEncoderValue(){
        return climber.getEncoder().getPosition();
    }

    public void climber0(){
        climber.getClosedLoopController().setReference(0, ControlType.kPosition);
    }
     
     public void dumbClimb(){
         climber.set(-0.2);
     }

     public void dumbClimbComp(){
        climber.set(0.2);
     }

    public void resetEncoder(){
        climber.getEncoder().setPosition(0);
    }

    public void climberStop(){
        climber.set(0);
    }
    public boolean isDone(Stage stage){
        if(climber.getEncoder().getPosition() >= stage.encoderValue){
            return true;
        } else {
            return false;
        }

        
    }

    public void climb(){
        climber.set(0.5);
    }

        
    @Override
    public void periodic() {
        SmartDashboard.putNumber( "ClimberEncoderValue", this.getEncoderValue());
    }

}

    
