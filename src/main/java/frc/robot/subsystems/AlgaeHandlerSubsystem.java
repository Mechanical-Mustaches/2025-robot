package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.ArrayList;
import java.util.Map;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DumbAlgaePivotCommand;

public class AlgaeHandlerSubsystem extends SubsystemBase {
    private final static double ALGAE_DETECTION_THRESHOLD = 10;
    private static final long MEASUREMENT_WINDOW = 500;
    // this number represents the minimum number of recent measurements needed to
    // calculate average amperage
    private static final int RECENT_MEASUREMENT_COUNT = 5;

    private static final Map<Position, ClosedLoopSlot> closedLoopSlots = Map.of(
            Position.In, ClosedLoopSlot.kSlot0,
            Position.Out, ClosedLoopSlot.kSlot1);

    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(22, MotorType.kBrushless);
    private boolean intakingAlgae = false;
    private ArrayList<AmperageMeasurements> amperageMeasurements = new ArrayList<AmperageMeasurements>();

    private record AmperageMeasurements(long time, double amperage) {
        public boolean isRecent(long milliseconds) {
            return System.currentTimeMillis() - time <= milliseconds;
        }
    }

    // private Encoder pivotEncoder = new Encoder(null, null)

    /**
     * Holds two algae pivot positions: in and out.
     */
    public enum Position {
        In(0.015),
        Out(0.076);

        private final double encoderValue;

        private Position(double encoderValue) {
            this.encoderValue = encoderValue;
        }

        public double getValue() {
            return this.encoderValue;
        }

        @Override
        public String toString() {
            switch (this) {
                case In:
                    return "In";

                case Out:
                    return "Out";
                default:
                    return "unknown";
            }
        }

    }

    public AlgaeHandlerSubsystem() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();

        this.setDefaultCommand(new DumbAlgaePivotCommand(this, Position.In));

        intakeConfig
                .smartCurrentLimit(10)
                .idleMode(IdleMode.kBrake);

        closedLoopConfig
                .pid(2, 0.000001, 0, closedLoopSlots.get(Position.In))
                .pid(2, 0.000001, 0, closedLoopSlots.get(Position.Out))
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        pivotConfig
                .apply(closedLoopConfig)
                .smartCurrentLimit(15)
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
        intakeActivator.set(1);
    }

    public void hold() {
        intakeActivator.set(0.1);
    }

    public void launch() {
        intakeActivator.set(-1);
    }

    public void stopIntake() {
        intakingAlgae = false;
        intakeActivator.set(0);
    }

    public void pivot(Position targetPosition) {
        var reference = targetPosition.getValue();
        var slot = closedLoopSlots.get(targetPosition);
        SmartDashboard.putNumber("algae/target", reference);

        pivot.getClosedLoopController().setReference(reference, ControlType.kPosition, slot);

    }

    public boolean isAlgaeDetected() {
        return getAverageAmperage() > ALGAE_DETECTION_THRESHOLD;
    }

    public boolean isIntakingAlgae() {
        return intakingAlgae;
    }

    private AmperageMeasurements getCurrentMeasurement() {
        return new AmperageMeasurements(System.currentTimeMillis(), intakeActivator.getOutputCurrent());
    }

    private double getAverageAmperage() {
        if (amperageMeasurements.isEmpty()) {
            return 0;
        }

        var recentMeasurementCount = 0;
        double sumOfElements = 0;

        for (var measurement : amperageMeasurements) {
            sumOfElements = sumOfElements + measurement.amperage;
            if (measurement.isRecent(MEASUREMENT_WINDOW)) {
                recentMeasurementCount = recentMeasurementCount + 1;
            }
        }

        if (recentMeasurementCount < RECENT_MEASUREMENT_COUNT) {
            return 0;
        }

        return sumOfElements / amperageMeasurements.size();
    }

    private void removeMeasurements() {
        amperageMeasurements.removeIf(measurement -> !measurement.isRecent(MEASUREMENT_WINDOW));
    }

    public void reset() {
        amperageMeasurements.clear();
        stopIntake();
    }

    @Override
    public void periodic() {

        removeMeasurements();
        amperageMeasurements.add(getCurrentMeasurement());

        SmartDashboard.putNumber("intakeMotorAmperage", getAverageAmperage());
        SmartDashboard.putBoolean("algaeDetectionValue", isAlgaeDetected());
        SmartDashboard.putNumber("pivotEncoderValue", pivot.getAbsoluteEncoder().getPosition());
        SmartDashboard.putBoolean("intakingAlgae", intakingAlgae);
        SmartDashboard.putNumber("AlgaePivotCurrentDraw", pivot.getOutputCurrent());
        SmartDashboard.putNumber("PivotRelativeEncoderValue", pivot.getEncoder().getPosition());

    }

}
