// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

// import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController.ArbFFUnits;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ArmIOSparkMax implements ArmIO {

  private final SparkMax leader =
      new SparkMax(frc.robot.Constants.LEFT_WORM_GEAR, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(frc.robot.Constants.RIGHT_WORM_GEAR, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private static RelativeEncoder encoder;
  private SparkClosedLoopController pid;
  private final SparkMaxConfig leaderConfig;
  public ArmIOSparkMax() {

    encoder = leader.getEncoder();
    // encoder.setPositionConversionFactor(Constants.ARM_GEAR_RATIO);
    encoder.setPosition(Constants.ANGLE_START_POSITION);
    follower.getEncoder().setPosition(Constants.ANGLE_START_POSITION);

    pid = leader.getClosedLoopController();
    // pid.setOutputRange(-5, 100);

    leaderConfig =
        (SparkMaxConfig) new SparkMaxConfig()
            
            .smartCurrentLimit(45)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(1.0 / 1.5)
                    .velocityConversionFactor(1.0 / (60.0 * 1.5)))
            .inverted(false);

    leader.configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    follower.configure(new SparkMaxConfig().follow(leader.getDeviceId()).inverted(false),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);

    Timer.delay(1);

    
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    // inputs.velocityRadPerSec =
    //     Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
    inputs.position[0] = leader.getEncoder().getPosition();
    inputs.position[1] = follower.getEncoder().getPosition();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  /*@Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setReference(
        Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec) * GEAR_RATIO,
        ControlType.kVelocity,
        0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }
  */
  @Override
  public void setPosition(double position) {
    System.out.println(position);
    leader.getClosedLoopController().setReference(Rotations.convertFrom(position, Radians), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leader.configure(leaderConfig.apply(new ClosedLoopConfig().pid(kP,kI,kD)), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public static void setEncoders() {
    encoder.setPosition(frc.robot.Constants.ANGLE_AMP);
  }

  public static RelativeEncoder getEncoder() {
    return encoder;
  }
}
