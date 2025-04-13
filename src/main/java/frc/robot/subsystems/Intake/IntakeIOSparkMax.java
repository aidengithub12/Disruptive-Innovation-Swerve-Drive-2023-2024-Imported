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

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 1.5;

  private final SparkMax leader =
      new SparkMax(frc.robot.Constants.INTAKE, MotorType.kBrushless);
  // private final CANSparkMax follower = new CANSparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();
  private final SparkMaxConfig leaderconfig;
  private double velocitySetpoint = 0.0;
  public IntakeIOSparkMax() {
    leaderconfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(20)
      .apply(new EncoderConfig().positionConversionFactor(1 / GEAR_RATIO).velocityConversionFactor(1 / (60 * GEAR_RATIO)));
    leader.configure(leaderconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);  
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    velocitySetpoint = velocityRadPerSec;

    leader
        .getClosedLoopController()
        .setReference(
            velocitySetpoint,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leader.configure(
        leaderconfig.apply(new ClosedLoopConfig().pid(kP, kI, kD)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }
}
