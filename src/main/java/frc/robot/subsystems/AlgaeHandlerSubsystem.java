package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeHandlerSubsystem extends SubsystemBase {
    private SparkMax intakeActivator = new SparkMax(23, MotorType.kBrushless);
    private SparkMax pivot = new SparkMax(22, MotorType.kBrushless);
    private boolean intakingAlgae = false;
    // private Encoder pivotEncoder = new Encoder(null, null)

    public enum Position {
        in(0),
        out(0.25);

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

        intakeConfig
                .smartCurrentLimit(10)
                .idleMode(IdleMode.kBrake);

        pivotConfig
                //.smartCurrentLimit(10)
                .idleMode(IdleMode.kBrake);

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

    public void pivotOut() {
        if (pivot.getAbsoluteEncoder().getPosition() < 0.25) {
            pivot.set(0.3);
         } else{
            stopPivot();
         }
    }

    public void pivotIn() {
        if (pivot.getAbsoluteEncoder().getPosition() > 0) {
            pivot.set(-0.3);
         } else{
            stopPivot();
         }
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
        SmartDashboard.putNumber("pivotEncoderValue", pivot.getEncoder().getPosition());
        SmartDashboard.putBoolean("intakingAlgae", intakingAlgae);
        SmartDashboard.putNumber("AlgaeCurrentDraw", intakeActivator.getOutputCurrent());
        SmartDashboard.putNumber("PivotRelativeEncoderValue", pivot.getEncoder().getPosition());

    }

}
