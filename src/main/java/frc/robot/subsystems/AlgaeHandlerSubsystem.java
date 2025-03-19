package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase {
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(22, MotorType.kBrushless);
    private boolean intakingAlgae = false;

    private record AmperageMeasurements(long time, double amperage) {
        public boolean isRecent(long window) {
            return System.currentTimeMillis() - time <= window;
        }
    }

    // private Encoder pivotEncoder = new Encoder(null, null)

    /**
     * Holds two algae pivot positions: in and out.
     */
    public enum Position {
        In(0),
        Out(0.25);

        private final double encoderValue;

        private Position(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getValue() {
            return this.encoderValue;
        }
    }

    public AlgaeHandlerSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

        intakeConfig
                .smartCurrentLimit(10)
                .idleMode(IdleMode.kBrake);

        closedLoopConfig
                .pid(0.3, 0.000001, 0)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        pivotConfig
                .apply(closedLoopConfig)
                .idleMode(IdleMode.kBrake);

        pivot.configure(pivotConfig, null, null);
    }

    public double getEncoderValue() {
        return pivot.getAbsoluteEncoder().getPosition();
    }

    public void stopPivot() {
        pivot.set(0);
    }

    public void resetEncoder() {
        pivot.getEncoder().setPosition(0);
    }

    public void intake() {
        intakingAlgae = true;
        // if (intakeActivator.getOutputCurrent() < 6) {
        // intakeActivator.set(1);
        // } else {
        // intakeActivator.set(0.1);
        // }

        if (isAlgaeDetected() > 5) {
            intakeActivator.set(0.1);
        } else {
            intakeActivator.set(1);
        }

    }

    public void launch() {
        intakeActivator.set(-1);
    }

    public void stopIntake() {
        intakingAlgae = false;
        intakeActivator.set(0);
    }

    public void pivot(Position targetPosition) {
        pivot.getClosedLoopController().setReference(targetPosition.getValue(), ControlType.kPosition);
    }

    public double isAlgaeDetected() {
        if (intakeActivator.getOutputCurrent() > 15) {
            return intakeActivator.getOutputCurrent() - 6;
        } else {
            return 0;
        }
    }

    public boolean isIntakingAlgae() {
        return intakingAlgae;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("algaeDetectionValue", isAlgaeDetected());
        SmartDashboard.putNumber("pivotEncoderValue", pivot.getAbsoluteEncoder().getPosition());
        SmartDashboard.putBoolean("intakingAlgae", intakingAlgae);
        SmartDashboard.putNumber("AlgaeCurrentDraw", intakeActivator.getOutputCurrent());
        SmartDashboard.putNumber("PivotRelativeEncoderValue", pivot.getEncoder().getPosition());

    }

}
