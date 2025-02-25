package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase{

    EndEffectorSubsystem endEffector; 

    public enum Level{
        L1(0),
        L2(6.5),
        L3(20.5),
        L4(46),
        //previously 46
        LIntake(1);

        public final double encoderValue;

        private Level(double level){
            this.encoderValue = level;
        }
    }

    
    

    private SparkMax leftEleMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightEleMotor = new SparkMax(17, MotorType.kBrushless);

    public ElevatorSubsystem(){
        
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        
        

        

        pidConfig
            .pid(0.05,0.00001,0.00006,ClosedLoopSlot.kSlot0)
            //.maxOutput(.6,ClosedLoopSlot.kSlot0)
            //.minOutput(0,ClosedLoopSlot.kSlot0)
            
            .pid(0.05,0.00001,0.00006,ClosedLoopSlot.kSlot1)
           // .maxOutput(0,ClosedLoopSlot.kSlot1)
            //.minOutput(-0.6,ClosedLoopSlot.kSlot1)

            //.pid(0.01,0.0000001,0.00006,ClosedLoopSlot.kSlot2)
            .pid(0.0,0.000000,0.0000,ClosedLoopSlot.kSlot2)

            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

       

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig)
            .inverted(true);

        rightConfig
            .apply(leftConfig)
            .follow(16,true);

        leftEleMotor.configure(leftConfig,null,null);
        rightEleMotor.configure(rightConfig,null,null);
    }

   

    public void setPosition(Level targetLevel, EndEffectorSubsystem endEffector){
        this.endEffector = endEffector;
        // if (getEncoderValue() < 2.5){
        //     if (endEffector.isCoralSeenFront()){
        //         if(getEncoderValue() > targetLevel.encoderValue){
        //             leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot1);
        //         } else {
        //             leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        //         }
        // }
        
        // }else{
        //     if(getEncoderValue() > targetLevel.encoderValue){
        //         leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot1);
        //     } else {
        //         leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        //     }
        // }
        

        if(getEncoderValue() > targetLevel.encoderValue){
                     leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot1);
                 } else {
                     leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot0);
                 }
    }

    public void adjust(boolean up){
        if (up){
        leftEleMotor.set(0.2);
        } else {
            leftEleMotor.set(-0.2);
        }
        
    }
    
    
    public double getEncoderValue(){
        return leftEleMotor.getEncoder().getPosition();
    }
    public void stopElevator(){
        leftEleMotor.set(0);
    } public void clearPosition(){
        //TODO: clear set position on enabling
    }
    public void algaeDescent(){
        leftEleMotor.getClosedLoopController().setReference(Level.L1.encoderValue, 
        ControlType.kPosition,ClosedLoopSlot.kSlot2);
                 
    }

}

