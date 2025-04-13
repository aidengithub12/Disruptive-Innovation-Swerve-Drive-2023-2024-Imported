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

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.util.SparkUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private  StatusSignal<Double> turnAbsolutePosition;
  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;
  private final SparkMaxConfig turnConfig;
  private final SparkMaxConfig driveConfig;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new SparkMax(Constants.DRIVE_FRONT_LEFT, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.TURN_FRONT_LEFT, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.DRIVE_ENCODER_FL);
        absoluteEncoderOffset = new Rotation2d(-2.534); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new SparkMax(Constants.DRIVE_FRONT_RIGHT, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.TURN_FRONT_RIGHT, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.DRIVE_ENCODER_FR);
        absoluteEncoderOffset = new Rotation2d(-2.640); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new SparkMax(Constants.DRIVE_BACK_LEFT, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.TURN_BACK_LEFT, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.DRIVE_ENCODER_BL);
        absoluteEncoderOffset = new Rotation2d(-1.097); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new SparkMax(Constants.DRIVE_BACK_RIGHT, MotorType.kBrushless);
        turnSparkMax = new SparkMax(Constants.TURN_BACK_RIGHT, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(Constants.DRIVE_ENCODER_BR);
        absoluteEncoderOffset = new Rotation2d(-3.938); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    driveController = driveSparkMax.getClosedLoopController();
    turnController = turnSparkMax.getClosedLoopController();

    // Configure drive motor
    driveConfig = (SparkMaxConfig) DriveConstantsNew.driveMotorConfig;
    driveConfig.inverted(false); //TODO: check inverts
    frc.robot.util.SparkUtil.tryUntilOk(
        driveSparkMax,
        50,
        () ->
            driveSparkMax.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
                frc.robot.util.SparkUtil.tryUntilOk(driveSparkMax, 5, () -> driveEncoder.setPosition(0.0));

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = DriveConstantsNew.cancoderConfig;
    cancoderConfig.MagnetSensor.withMagnetOffset(absoluteEncoderOffset.getRotations());
    frc.robot.util.PhoenixUtil.tryUntilOk(5, ()->turnAbsoluteEncoder.getConfigurator().apply(cancoderConfig,0.25));
    
    
    
    
    
    // Configure turn motor
    turnConfig = (SparkMaxConfig) DriveConstantsNew.turnMotorConfig;
    turnConfig.inverted(false);
      
    
    frc.robot.util.SparkUtil.tryUntilOk(
        turnSparkMax,
        50,
        () ->
            turnSparkMax.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    


    frc.robot.util.SparkUtil.tryUntilOk(
        turnSparkMax,
        50,
        () -> turnRelativeEncoder.setPosition(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() *2 * Math.PI));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
    if (turnAbsolutePosition == null) {
      inputs.turnAbsolutePosition = new Rotation2d();
    }
    else {
      inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()).minus(absoluteEncoderOffset);
    }
    //  turnAbsolutePosition == null ?
    //     inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
    //         .minus(absoluteEncoderOffset) : inputs.turnAbsolutePosition = new Rotation2d(0);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.configure(new SparkMaxConfig().apply(driveConfig).idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.configure(new SparkMaxConfig().apply(turnConfig).idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast),ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
  }
}
