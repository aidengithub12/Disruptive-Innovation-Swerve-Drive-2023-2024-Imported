package frc.robot.subsystems.drive;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

public class DriveConstantsNew {
    public static final SparkBaseConfig driveMotorConfig =
        new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40)
            .voltageCompensation(12.0)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(2 * Math.PI * ModuleIOSparkMax.DRIVE_GEAR_RATIO)
                    .velocityConversionFactor((2 * Math.PI) / 60.0 /  ModuleIOSparkMax.DRIVE_GEAR_RATIO)
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2))
            .apply(
                new ClosedLoopConfig()
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pidf(
                        0, 0.0,
                        0, 0.0))
            .apply(
                new SignalsConfig()
                    .primaryEncoderPositionAlwaysOn(true)
                    .primaryEncoderPositionPeriodMs((int) (1000.0 / 100.0))
                    .primaryEncoderVelocityAlwaysOn(true)
                    .primaryEncoderVelocityPeriodMs(20)
                    .appliedOutputPeriodMs(20)
                    .busVoltagePeriodMs(20)
                    .outputCurrentPeriodMs(20));

    public static final SparkMaxConfig turnMotorConfig =
        (SparkMaxConfig) new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12.0)
        .apply(
            new EncoderConfig()
                .positionConversionFactor(2 * Math.PI / ModuleIOSparkMax.TURN_GEAR_RATIO)
                .velocityConversionFactor((2 * Math.PI) / 60.0 / ModuleIOSparkMax.TURN_GEAR_RATIO)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2))
        .apply(
            new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .pidf(0, 0.0, 0, 0.0)) //TODO: find PID Values for drive and steer
        .apply(
            new SignalsConfig()
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / 100.0))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20));

    public static final CANcoderConfiguration cancoderConfig =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
}