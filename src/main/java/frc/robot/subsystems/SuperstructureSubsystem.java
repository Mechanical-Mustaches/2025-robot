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
    private boolean trapdoorReleased = false;

    SparkMaxConfig trapdoorConfig = new SparkMaxConfig();

    

    public enum Stage{
        Unknown(Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY),
        Closed(0,0),
        S1(-0.36,-0.19),
        S2(0.21,-0.19);
        
        private static final double tolerance = 0.1;

        public final double rightEncoderValue;
        public final double leftEncoderValue;

        private Stage(double left, double right){
            this.leftEncoderValue = left;
            this.rightEncoderValue = right;
        }

        

        public String toString(){
            if (this == Closed) {
                return "Closed";
            } else if(this == S1){
                return "S1";
            } else if(this == S2){
                return "S2";
            } else{
                return "Unknown";
            }
            

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

        
        public boolean equals(Stage other) {
            return this.equals(other.leftEncoderValue, other.rightEncoderValue);
        }

        public boolean equals(double left, double right){
            double leftDifference = Math.abs((this.leftEncoderValue-left));
            double rightDifference = Math.abs((this.rightEncoderValue-right));
            return(leftDifference<= tolerance && rightDifference<=tolerance);
        }

        public static Stage fromValues(double left, double right){
            if(Closed.equals(left, right)) {
                return Closed;
            }
            if(S1.equals(left, right)) {
                return S1;
            }
            if(S2.equals(left, right)) {
                return S2;
            } 

            return Unknown;
        }
    
    }
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
    public void toStage(Stage stage){
        leftPivot.set(0);
        rightPivot.set(0);
        rightPivot.getClosedLoopController().setReference(stage.rightEncoderValue, ControlType.kPosition);
        leftPivot.getClosedLoopController().setReference(stage.leftEncoderValue, ControlType.kPosition);
        trapdoorReleased = true;
    }

    public Stage getStage(){
        return Stage.fromValues(getLeftEncoderValue(), getRightEncoderValue());
    }

    public void stop(SparkMax motor){
        motor.set(0);
        
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
        
        SmartDashboard.putNumber("SuperstructureLeftEncoderValue", getLeftEncoderValue());
        SmartDashboard.putNumber("SuperstructureRightEncoderValue", getRightEncoderValue());
        SmartDashboard.putString("SuperstructureStage", Stage.fromValues(getLeftEncoderValue(), getRightEncoderValue()).toString());

        // if(!trapdoorReleased){
        //     leftPivot.set(-0.1);
        //     rightPivot.set(0.1);
        // }
    }





}
