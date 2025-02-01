package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase{

    public enum Level{
        L1(0),
        L2(12),
        L3(24),
        L4(36);

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
            .pid(0.02,0,0,ClosedLoopSlot.kSlot0)
            .maxOutput(.6,ClosedLoopSlot.kSlot0)
            .minOutput(0.1,ClosedLoopSlot.kSlot0)
            
            .pid(0.01,0,0,ClosedLoopSlot.kSlot1)
            .maxOutput(-0.1,ClosedLoopSlot.kSlot1)
            .minOutput(-0.6,ClosedLoopSlot.kSlot1);

       

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

    
    

    public void setPosition(Level targetLevel){
       
        if(getEncoderValue() > targetLevel.encoderValue){
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot1);
        } else {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,ClosedLoopSlot.kSlot0);
        }
    }

    public void elevatorUp(){
        leftEleMotor.getClosedLoopController().setReference(36, ControlType.kPosition);
        
    }
    public void elevatorDown(){
        leftEleMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
    }
    public double getEncoderValue(){
        return leftEleMotor.getEncoder().getPosition();
    }
    public void stopElevator(){
        leftEleMotor.set(0);
    } public void clearPosition(){
        //TODO: clear set position on enabling
    }

}
