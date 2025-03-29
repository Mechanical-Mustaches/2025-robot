package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperstructureSubsystem extends SubsystemBase {
    private SparkMax leftPivot = new SparkMax(20, MotorType.kBrushless);
    private SparkMax rightPivot = new SparkMax(21, MotorType.kBrushless);
    private boolean isTeleOp = false;
    private boolean isOpen = false;
    private boolean isEnabled = false;

    public SuperstructureSubsystem() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig.pid(5, 0.00005, 0.00003);

        leftConfig
                .smartCurrentLimit(30)
                .idleMode(IdleMode.kBrake)
                .apply(pidConfig);

        rightConfig
                .apply(leftConfig);

        leftPivot.configure(leftConfig, null, null);
        rightPivot.configure(leftConfig, null, null);
    }

    public void keepClosed() {
        leftPivot.set(0.2);
        rightPivot.set(0.2);
    }

    public void keepClosedStop() {
        leftPivot.set(0);
        rightPivot.set(0);
    }

    public SparkMax getLeftMotor() {
        return leftPivot;
    }

    public SparkMax getRightMotor() {
        return rightPivot;
    }

    public void open() {
        isOpen = true;
    }

    public boolean isOpen() {
        return isOpen;
    }

    public void enable(){
        isEnabled = true;
    }

    public void reset(){
        isEnabled = false;
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous() || !isEnabled) {
            keepClosed();
        }
    }
}
