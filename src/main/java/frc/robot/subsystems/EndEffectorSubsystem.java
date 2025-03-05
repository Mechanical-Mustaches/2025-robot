package frc.robot.subsystems;


import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {

    private SparkMax effectorLeft = new SparkMax(13, MotorType.kBrushless);
    private SparkMax effectorRight = new SparkMax(14, MotorType.kBrushless);
    private SparkLimitSwitch frontLineBreakSensor = effectorLeft.getForwardLimitSwitch();
    private SparkLimitSwitch backLineBreakSensor = effectorRight.getForwardLimitSwitch();

    public EndEffectorSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        rightConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
            
        effectorLeft.configure(leftConfig, null, null);
        effectorRight.configure(rightConfig, null, null);
    }
    public void effectorSpin(){
        effectorLeft.set(-0.2);
        effectorRight.set(0.2);
    }

    public void effectorScore(){
        effectorLeft.set(-0.4);
        effectorRight.set(0.4);
    }

    public void L1Score(){
        effectorLeft.set(-0.2);
        effectorRight.set(0.6);
    }

    public void L4Score(){
        effectorLeft.set(-0.18);
        effectorRight.set(0.18);
    }

    public void effectorInverse(){
        effectorLeft.set(0.2);
    }
   
    public void effectorStop(){
        effectorLeft.set(0);
        effectorRight.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("coral detection front", frontLineBreakSensor.isPressed());
        SmartDashboard.putBoolean("coral detection back", backLineBreakSensor.isPressed());
    }
    public boolean isCoralSeenFront(){
        return frontLineBreakSensor.isPressed();
    }
    public boolean isCoralSeenBack(){
        return backLineBreakSensor.isPressed();
    }
}
