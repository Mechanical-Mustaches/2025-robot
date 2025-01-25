package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    public enum Level{
        L1,
        L2,
        L3,
        L4
    }

    private SparkMax leftEleMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightEleMotor = new SparkMax(17, MotorType.kBrushless);

    public ElevatorSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
            .pid(0.02,0,0)
            .maxOutput(.6)
            .minOutput(.1);

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig)
            .inverted(true);

        rightConfig
            .apply(leftConfig)
            .follow(16,true);

        leftEleMotor.configure(leftConfig,null,null);
        rightEleMotor.configure(leftConfig,null,null);
    }

    public void setPosition(Level level){
        if (level == Level.L1){
            leftEleMotor.getClosedLoopController().setReference(0, ControlType.kPosition);  
        } else if (level == Level.L2){
            leftEleMotor.getClosedLoopController().setReference(12, ControlType.kPosition);  
        } else if (level == Level.L3){
            leftEleMotor.getClosedLoopController().setReference(24, ControlType.kPosition);  
        } else if (level == Level.L4){
            leftEleMotor.getClosedLoopController().setReference(36, ControlType.kPosition);  
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
