package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase{
    private SparkMax leftPivot = new SparkMax(20,MotorType.kBrushless);
    private SparkMax rightPivot = new SparkMax(21,MotorType.kBrushless);
    

    public SuperstructureSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();


        pidConfig
            .pid(0.01, 0, 0);
            

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig);
        
        rightConfig
            .apply(leftConfig);
            
            


            leftPivot.configure(leftConfig,null,null);
            rightPivot.configure(leftConfig,null,null);
    }

    public void stageOne(){
        
        rightPivot.set(0.2);
        leftPivot.set(-0.2);
    }


    public int getStage(){
        return 1;
    }
    public void stop(){
        leftPivot.set(0);
    }

    public double getLeftEncoderValue(){
        return leftPivot.getEncoder().getPosition();
    }
    public double getRightEncoderValue(){
        return rightPivot.getEncoder().getPosition();
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("SuperLeftStructureEncoderValue", getLeftEncoderValue());
        SmartDashboard.putNumber("SuperRightStructureEncoderValue", getRightEncoderValue());
    }



}
