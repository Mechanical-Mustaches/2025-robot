package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase{
    private SparkMax leftPivot = new SparkMax(20,MotorType.kBrushless);
    private SparkMax rightPivot = new SparkMax(21,MotorType.kBrushless);
    private Stage currentStage = Stage.Closed;

    public enum Stage{
        Closed(-0.52,0.5),
        S1(-0.62,0.31),
        S2(-0.05,0.21);
        

        public final double rightEncoderValue;
        public final double leftEncoderValue;

        private Stage(double left, double right){
            this.leftEncoderValue = left;
            this.rightEncoderValue = right;
        }

        public boolean isValidTarget(Stage target) {
            switch (target) {
                case S1:
                    return this == Closed;
                case S2:
                    return this == S1;

                default:
                    return false;
                    
            }
        }
    
    }
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

    public void toStage(Stage stage){
        if (!currentStage.isValidTarget(stage)) {
            return;
        }

        rightPivot.getClosedLoopController().setReference(stage.rightEncoderValue, ControlType.kPosition);
        leftPivot.getClosedLoopController().setReference(stage.leftEncoderValue, ControlType.kPosition);
        
        currentStage = stage;
    }


    public Stage getStage(){
        return currentStage;
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
