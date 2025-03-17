package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    EndEffectorSubsystem endEffector;

    public enum Level {
        LIntake(0),
        LAlgaeTop(0),
        LAlgaeBottom(0),
        L1(11.5),
        L2(19),
        L3(33),
        L4(57.5);

        public final double encoderValue;

        private Level(double level) {
            this.encoderValue = level;
        }
    }

    private SparkMax leftEleMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightEleMotor = new SparkMax(17, MotorType.kBrushless);

    public ElevatorSubsystem() {

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
                .pid(0.05, 0.00001, 0.00006, ClosedLoopSlot.kSlot0)
                .pid(0.05, 0.00001, 0.00006, ClosedLoopSlot.kSlot1)
                .pid(0.0, 0.000000, 0.0000, ClosedLoopSlot.kSlot2)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        leftConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake)
                .apply(pidConfig)
                .inverted(true);

        rightConfig
                .apply(leftConfig)
                .follow(16, true);

        leftEleMotor.configure(leftConfig, null, null);
        rightEleMotor.configure(rightConfig, null, null);
    }

    public void setPosition(Level targetLevel, EndEffectorSubsystem endEffector) {
        this.endEffector = endEffector;

        if (getEncoderValue() > targetLevel.encoderValue) {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,
                    ClosedLoopSlot.kSlot1);
        } else {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue, ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
        }
    }

    public void adjust(boolean up) {
        if (up) {
            leftEleMotor.set(0.2);
        } else {
            leftEleMotor.set(-0.2);
        }

    }

    public double getEncoderValue() {
        return leftEleMotor.getEncoder().getPosition();
    }

    public void stopElevator() {
        leftEleMotor.set(0);
    }

    public void algaeDescent() {
        leftEleMotor.getClosedLoopController().setReference(Level.L1.encoderValue,
                ControlType.kPosition, ClosedLoopSlot.kSlot2);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorEncoderValue", getEncoderValue());
    }

}
