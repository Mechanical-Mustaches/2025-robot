package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SuperstructureSubsystem;

public class SuperstructureMotorMove extends Command{
    SuperstructureSubsystem superstructureSubsystem;
    SparkMax motor;
    double power;
    boolean bumperState;

    public SuperstructureMotorMove(SuperstructureSubsystem subsystem, SparkMax motor, double power){
        this.superstructureSubsystem = subsystem;
        this.motor = motor;
        this.power = power;
        
    }

    @Override
    public void initialize() {
       
        superstructureSubsystem.moveMotor(motor, power);
       
    
        
        
        
    }
  
   
    @Override
    public void end(boolean interrupted) {
        superstructureSubsystem.stop(motor); 
    }
    
}
