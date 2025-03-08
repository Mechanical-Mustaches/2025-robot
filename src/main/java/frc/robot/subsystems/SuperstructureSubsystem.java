package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase{
    private SparkMax leftPivot = new SparkMax(20,MotorType.kBrushless);
    private SparkMax rightPivot = new SparkMax(21,MotorType.kBrushless);
    private boolean trapdoorReleased = false;
    private boolean isTeleOp = false;

    SparkMaxConfig trapdoorConfig = new SparkMaxConfig();

    public SuperstructureSubsystem(){
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
       
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
            .pid(5, 0.00005, 0.00003);
            

        leftConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .apply(pidConfig);
        
        rightConfig
            .apply(leftConfig);
            
            


            leftPivot.configure(leftConfig,null,null);
            rightPivot.configure(leftConfig,null,null);
        
    }

    
    public void moveMotor(SparkMax motor,double power){

        // trapdoorReleased = true;
        // trapdoorConfig
        // .idleMode(IdleMode.kBrake)
        // .smartCurrentLimit(50);
        // leftPivot.configure(trapdoorConfig,null,null);
        // rightPivot.configure(trapdoorConfig,null,null);
        motor.set(power);
    }

    public void stop(SparkMax motor){
        motor.set(0);
    }

    public void keepClosed(){
        leftPivot.set(0.1);
        rightPivot.set(0.1);
    }

    public void keepClosedStop(){
        leftPivot.set(0);
        rightPivot.set(0);
    }

    public void resetEncoders(){
        rightPivot.getEncoder().setPosition(0);
        leftPivot.getEncoder().setPosition(0);
    }
    public double getLeftEncoderValue(){
        return leftPivot.getEncoder().getPosition();
    }
    public double getRightEncoderValue(){
        return rightPivot.getEncoder().getPosition();
    }

    public SparkMax getLeftMotor(){
        return leftPivot;
    }
    public SparkMax getRightMotor(){
        return rightPivot;
    }
    public boolean getTrapdoorState(){
        return trapdoorReleased;
    }

    

    @Override
    public void periodic(){
        
        if (DriverStation.isAutonomous() || (DriverStation.getMatchTime()>40)){
            keepClosed();
        } else if (!isTeleOp) {
            isTeleOp = true;
            keepClosedStop();
        }
        SmartDashboard.putNumber("SuperstructureLeftEncoderValue", getLeftEncoderValue());
        SmartDashboard.putNumber("SuperstructureRightEncoderValue", getRightEncoderValue());
        SmartDashboard.putNumber("timer", DriverStation.getMatchTime()); 
        SmartDashboard.putBoolean("can doors open", DriverStation.isTeleop() && DriverStation.getMatchTime() < 130);
        
    }





}
