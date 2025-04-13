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

package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.*;
/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ShooterIOSparkMax implements ShooterIO {
  private static final double GEAR_RATIO = 1;

  private final SparkMax leader =
      new SparkMax(frc.robot.Constants.SHOOTER_TOP, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final SparkMax follower =
      new SparkMax(frc.robot.Constants.SHOOTER_BOTTOM, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoder = leader.getEncoder();
  private final SparkClosedLoopController pid = leader.getClosedLoopController();
  private final SparkClosedLoopController pid_follower = follower.getClosedLoopController();
  private final SparkBaseConfig leaderconfig;
  private double velocitySetpoint = 0.0;
  
  public ShooterIOSparkMax() {
    leaderconfig = new SparkMaxConfig().idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(20)
      .apply(new EncoderConfig().positionConversionFactor(1 / GEAR_RATIO).velocityConversionFactor(1 / (60 * GEAR_RATIO)));
    leader.configure(leaderconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);  
    follower.configure(new SparkMaxConfig().follow(leader.getDeviceId()).inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

  } 

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.positionRad = encoder.getPosition();
    inputs.velocityRPM = encoder.getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent(), follower.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
    follower.setVoltage(volts);
  }

  // @Override
  // public void setVelocityTopSlow(double velocityRPM, double ffVolts) {
  //   pid.setReference(
  //       velocityRPM * GEAR_RATIO, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
  // }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {
    velocitySetpoint = velocityRPM;
    leader.getClosedLoopController().setReference(velocitySetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0,ffVolts);
    //if follower does not follow leader right away
    // follower.getClosedLoopController().setReference(velocitySetpoint, ControlType.kVelocity, ClosedLoopSlot.kSlot0,ffVolts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leader.configure(leaderconfig.apply(new ClosedLoopConfig().pid(kP, kI, kD)), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }
}
