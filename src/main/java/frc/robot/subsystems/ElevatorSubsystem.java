package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    EndEffectorSubsystem endEffector;
    private static final double STAGE_ENCODER_OFFSET = 0;

    public enum Level {
        LIntake(0),
        LAlgaeTop(19.5),
        LAlgaeBottom(7.5),
        L1(12),
        L2(21),
        L3(34),
        L4(57.5);

        public final double encoderValue;

        private Level(double level) {
            this.encoderValue = level;
        }
    }

    private SparkMax leftEleMotor = new SparkMax(16, MotorType.kBrushless);
    private SparkMax rightEleMotor = new SparkMax(17, MotorType.kBrushless);
    private SparkLimitSwitch limitSwitch = leftEleMotor.getForwardLimitSwitch();

    public ElevatorSubsystem() {

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        ClosedLoopConfig pidConfig = new ClosedLoopConfig();

        pidConfig
                // .pid(0.03, 0.00003, 0.00006, ClosedLoopSlot.kSlot0)
                // .pid(0.02, 0.0000, 0.00003, ClosedLoopSlot.kSlot1)
                .pidf(0.03, 0.00001, 0.00006, 0.001, ClosedLoopSlot.kSlot0)
                .pidf(0.02, 0.00001, 0.00006, 0.001, ClosedLoopSlot.kSlot1)
                .pidf(0.05, 0.00001, 0.00006, 0.001, ClosedLoopSlot.kSlot2)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        leftConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake)
                .apply(pidConfig)
                .inverted(true);

        rightConfig
                .apply(leftConfig)
                .follow(16, true);

        leftEleMotor.configure(leftConfig, null, null);
        rightEleMotor.configure(rightConfig, null, null);
    }

    public void setPosition(Level targetLevel) {

        if (targetLevel == Level.LAlgaeBottom || targetLevel == Level.LAlgaeTop) {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue - STAGE_ENCODER_OFFSET,
                    ControlType.kPosition, ClosedLoopSlot.kSlot2);
        }

        if (getEncoderValue() > targetLevel.encoderValue) {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue - STAGE_ENCODER_OFFSET,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot1);
        } else {
            leftEleMotor.getClosedLoopController().setReference(targetLevel.encoderValue - STAGE_ENCODER_OFFSET,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
        }
    }

    public void reset() {
        if (DriverStation.getMatchType() == MatchType.None) {
            setPosition(Level.LIntake);
        }
    }

    public Level getNearestAlgaeLevel() {
        double offset = 5;
        if (getEncoderValue() >= Level.L2.encoderValue + offset) {
            return Level.LAlgaeTop;
        }

        return Level.LAlgaeBottom;
    }

    public void adjust(boolean up) {
        if (up) {
            leftEleMotor.set(0.2);
        } else {
            leftEleMotor.set(-0.2);
        }

    }

    public void dumbElevatorUp() {
        leftEleMotor.set(0.2);
    }

    public void dumbElevatorDown() {
        leftEleMotor.set(-0.2);
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

    public void resetEncoders() {
        leftEleMotor.getEncoder().setPosition(0);
        rightEleMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        if (limitSwitch.isPressed()) {
            resetEncoders();
        }
        SmartDashboard.putNumber("elevator/encoderValue", getEncoderValue());
        SmartDashboard.putNumber("dlevator/amperage", leftEleMotor.getOutputCurrent());
    }

}
